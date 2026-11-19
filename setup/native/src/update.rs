use std::time::Duration;

use anyhow::{Context, Result, bail, ensure};
use self_update::backends::github::{Update, UpdateBuilder};
use self_update::{ReleaseAsset, version::bump_is_greater};

use crate::service::ServiceDirectory;

const INSTALLED_VERSION: &str = env!("CARGO_PKG_VERSION");
const GITHUB_OWNER: &str = "Hebbian-Robotics";
const GITHUB_REPOSITORY: &str = "so101";

struct AvailableUpdate {
    version: String,
    archive_name: String,
    digest: String,
}

enum UpdateAvailability {
    Current { latest_version: String },
    Available(AvailableUpdate),
}

pub fn run(directory: &ServiceDirectory, check: bool) -> Result<()> {
    let archive_name = platform_archive(std::env::consts::OS, std::env::consts::ARCH)?;
    // The same lock covers hardware setup, servo maintenance, and replacement
    // so the executable cannot change during an active setup session.
    let _update_lock = if check {
        None
    } else {
        Some(
            directory
                .lock()
                .context("Stop the running so101 session before updating")?,
        )
    };
    let mut updater = Update::configure();
    updater
        .repo_owner(GITHUB_OWNER)
        .repo_name(GITHUB_REPOSITORY)
        .bin_name(GITHUB_REPOSITORY)
        .current_version(INSTALLED_VERSION)
        .timeout(Duration::from_secs(90));

    match check_for_update(&updater, INSTALLED_VERSION, &archive_name)? {
        UpdateAvailability::Current { latest_version } => {
            println!(
                "so101 {INSTALLED_VERSION} is up to date (latest published: {latest_version})."
            );
        }
        UpdateAvailability::Available(available) if check => {
            println!(
                "so101 {} is available (installed: {INSTALLED_VERSION}). Run so101 update to install it.",
                available.version
            );
        }
        UpdateAvailability::Available(available) => {
            println!(
                "Updating so101 from {INSTALLED_VERSION} to {}.",
                available.version
            );
            let installed = install_update(&mut updater, available)?;
            println!("Updated so101 to {installed}.");
        }
    }
    Ok(())
}

fn platform_archive(os: &str, architecture: &str) -> Result<String> {
    let architecture = match architecture {
        "x86_64" => "x64",
        "aarch64" => "arm64",
        _ => bail!("GitHub release archives are unavailable for {os}/{architecture}"),
    };
    ensure!(
        matches!(os, "linux" | "macos" | "windows"),
        "GitHub release archives are unavailable for {os}/{architecture}"
    );
    Ok(format!("so101-{os}-{architecture}.tar.gz"))
}

fn check_for_update(
    updater: &UpdateBuilder,
    installed_version: &str,
    archive_name: &str,
) -> Result<UpdateAvailability> {
    // GitHub's /releases/latest endpoint selects the latest published stable release.
    let releases = updater.build()?.get_latest_release()?;
    let release = releases
        .latest()
        .context("No published SO-101 release found")?;
    let version = release.version();
    if !bump_is_greater(installed_version, version)? {
        return Ok(UpdateAvailability::Current {
            latest_version: version.to_owned(),
        });
    }

    let archive = exact_archive(release.assets(), archive_name)
        .with_context(|| format!("SO-101 v{version} has no {archive_name} archive"))?;
    let digest = archive
        .digest()
        .filter(|digest| is_sha256(digest))
        .with_context(|| {
            format!("GitHub did not publish a valid SHA-256 digest for {archive_name}")
        })?;
    Ok(UpdateAvailability::Available(AvailableUpdate {
        version: version.to_owned(),
        archive_name: archive_name.to_owned(),
        digest: digest.to_owned(),
    }))
}

fn exact_archive<'a>(assets: &'a [ReleaseAsset], archive_name: &str) -> Option<&'a ReleaseAsset> {
    assets.iter().find(|asset| asset.name() == archive_name)
}

fn is_sha256(digest: &str) -> bool {
    digest
        .strip_prefix("sha256:")
        .is_some_and(|hash| hash.len() == 64 && hash.bytes().all(|digit| digit.is_ascii_hexdigit()))
}

fn install_update(updater: &mut UpdateBuilder, available: AvailableUpdate) -> Result<String> {
    let AvailableUpdate {
        version,
        archive_name,
        digest,
    } = available;
    let status = updater
        .release_tag(format!("v{version}"))
        .asset_matcher(move |assets| {
            exact_archive(assets, &archive_name)
                .filter(|asset| asset.digest() == Some(digest.as_str()))
                .cloned()
        })
        .verify_release_digest(true)
        .check_install_path_writable(true)
        .no_confirm(true)
        .build()?
        .update()
        .with_context(|| format!("Could not install SO-101 v{version}"))?;
    Ok(status.version().to_owned())
}

#[cfg(test)]
mod tests;
