use std::fmt::Write as _;
use std::io::Cursor;
use std::process::{Command, Output};
use std::sync::{
    Arc,
    atomic::{AtomicUsize, Ordering},
};

use anyhow::{Result, ensure};
use axum::{Json, Router, body::Bytes, extract::State, routing::get};
use flate2::{Compression, write::GzEncoder};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};

use super::{Update, UpdateAvailability, check_for_update, install_update, platform_archive};

const REPO: &str = "/repos/Hebbian-Robotics/so101";
const FIXTURE_URL: &str = "SO101_UPDATER_TEST_FIXTURE";
const FIXTURE_ACTION: &str = "SO101_UPDATER_TEST_ACTION";
const REPLACEMENT: &[u8] = b"fixture replacement for the running executable";

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn updater_replaces_a_running_copy_only_after_verifying_its_platform_archive() -> Result<()> {
    let archive_name = platform_archive(std::env::consts::OS, std::env::consts::ARCH)?;
    if let Some(url) = std::env::var_os(FIXTURE_URL) {
        let url = url
            .into_string()
            .map_err(|_| anyhow::anyhow!("Invalid fixture URL"))?;
        return tokio::task::spawn_blocking(move || exercise_updater(&url, &archive_name)).await?;
    }

    let directory = tempfile::tempdir()?;
    let current_binary = std::env::current_exe()?;
    let candidate = directory
        .path()
        .join(format!("candidate{}", std::env::consts::EXE_SUFFIX));
    std::fs::copy(&current_binary, &candidate)?;
    let original = std::fs::read(&candidate)?;

    let archive = make_archive()?;
    let mut digest = String::from("sha256:");
    for byte in Sha256::digest(&archive) {
        write!(digest, "{byte:02x}")?;
    }

    let downloads = Arc::new(AtomicUsize::new(0));
    let fixture = Fixture::serve(
        &archive_name,
        Some(&digest),
        &archive,
        Arc::clone(&downloads),
    )
    .await?;
    let checked = run_child(&candidate, &fixture.url, "check").await?;
    assert_success(&checked)?;
    assert_eq!(
        downloads.load(Ordering::SeqCst),
        0,
        "check downloaded the archive"
    );

    let missing_digest =
        Fixture::serve(&archive_name, None, &archive, Arc::clone(&downloads)).await?;
    let rejected = run_child(&candidate, &missing_digest.url, "install").await?;
    assert!(!rejected.status.success() && transcript(&rejected).contains("valid SHA-256 digest"));
    assert_eq!(
        downloads.load(Ordering::SeqCst),
        0,
        "metadata without a digest triggered a download"
    );
    ensure!(
        std::fs::read(&candidate)? == original,
        "Rejected update changed the executable"
    );

    let wrong_digest = format!("sha256:{}", "0".repeat(64));
    let corrupted = Fixture::serve(
        &archive_name,
        Some(&wrong_digest),
        &archive,
        Arc::clone(&downloads),
    )
    .await?;
    let rejected = run_child(&candidate, &corrupted.url, "install").await?;
    assert!(
        !rejected.status.success(),
        "a mismatched digest was accepted"
    );
    assert_eq!(downloads.load(Ordering::SeqCst), 1);
    ensure!(
        std::fs::read(&candidate)? == original,
        "Rejected update changed the executable"
    );

    let installed = run_child(&candidate, &fixture.url, "install").await?;
    assert_success(&installed)?;
    assert_eq!(downloads.load(Ordering::SeqCst), 2);
    assert_eq!(std::fs::read(&candidate)?, REPLACEMENT);
    Ok(())
}

fn exercise_updater(url: &str, archive_name: &str) -> Result<()> {
    let mut updater = Update::configure();
    updater
        .repo_owner("Hebbian-Robotics")
        .repo_name("so101")
        .bin_name("so101")
        .current_version("1.0.0")
        .ureq_agent(
            ureq::Agent::config_builder()
                .proxy(None)
                .timeout_global(Some(std::time::Duration::from_secs(10)))
                .build()
                .new_agent(),
        )
        .api_base_url(url);
    let UpdateAvailability::Available(release) = check_for_update(&updater, "1.0.0", archive_name)?
    else {
        anyhow::bail!("Fixture release was not offered");
    };
    if std::env::var(FIXTURE_ACTION)?.as_str() == "install" {
        ensure!(
            install_update(&mut updater, release)? == "1.1.0",
            "Wrong installed version"
        );
    }
    Ok(())
}

async fn run_child(candidate: &std::path::Path, url: &str, action: &'static str) -> Result<Output> {
    let candidate = candidate.to_owned();
    let url = url.to_owned();
    Ok(tokio::task::spawn_blocking(move || {
        Command::new(candidate)
            .args(["--exact", "update::tests::updater_replaces_a_running_copy_only_after_verifying_its_platform_archive"])
            .env(FIXTURE_URL, url)
            .env(FIXTURE_ACTION, action)
            .output()
    }).await??)
}

fn assert_success(output: &Output) -> Result<()> {
    ensure!(
        output.status.success(),
        "Updater child failed: {}",
        transcript(output)
    );
    Ok(())
}

fn transcript(output: &Output) -> String {
    format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    )
}

fn make_archive() -> Result<Vec<u8>> {
    let mut archive = tar::Builder::new(GzEncoder::new(Vec::new(), Compression::fast()));
    let mut header = tar::Header::new_gnu();
    header.set_size(u64::try_from(REPLACEMENT.len())?);
    header.set_mode(0o755);
    archive.append_data(
        &mut header,
        format!("./so101{}", std::env::consts::EXE_SUFFIX),
        Cursor::new(REPLACEMENT),
    )?;
    Ok(archive.into_inner()?.finish()?)
}

#[derive(Clone)]
struct FixtureState {
    release: Value,
    archive: Bytes,
    downloads: Arc<AtomicUsize>,
}

struct Fixture {
    url: String,
    server: tokio::task::JoinHandle<()>,
}

impl Fixture {
    async fn serve(
        archive_name: &str,
        digest: Option<&str>,
        archive: &[u8],
        downloads: Arc<AtomicUsize>,
    ) -> Result<Self> {
        let listener = tokio::net::TcpListener::bind((std::net::Ipv4Addr::LOCALHOST, 0)).await?;
        let url = format!("http://{}", listener.local_addr()?);
        let route = Router::new()
            .route(&format!("{REPO}/releases/latest"), get(release))
            .route(&format!("{REPO}/releases/tags/v1.1.0"), get(release))
            .route("/archive", get(download))
            .with_state(FixtureState {
                release: json!({
                    "name": "SO-101 fixture", "tag_name": "v1.1.0",
                    "created_at": "2026-01-01T00:00:00Z",
                    "assets": [
                        { "name": format!("extra-{archive_name}"), "url": format!("{url}/wrong") },
                        { "name": archive_name, "url": format!("{url}/archive"), "digest": digest }
                    ]
                }),
                archive: Bytes::copy_from_slice(archive),
                downloads,
            });
        let server = tokio::spawn(async move {
            let _ = axum::serve(listener, route).await;
        });
        Ok(Self { url, server })
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        self.server.abort();
    }
}

async fn release(State(fixture): State<FixtureState>) -> Json<Value> {
    Json(fixture.release)
}

async fn download(State(fixture): State<FixtureState>) -> Bytes {
    fixture.downloads.fetch_add(1, Ordering::SeqCst);
    fixture.archive
}
