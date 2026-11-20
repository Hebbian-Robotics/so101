use std::convert::Infallible;
use std::io::Write;
use std::net::{Ipv4Addr, SocketAddrV4};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{Context, Result, ensure};
use axum::body::Body;
use axum::extract::{DefaultBodyLimit, Request, State};
use axum::http::{HeaderMap, HeaderValue, Method, StatusCode, header};
use axum::middleware::{self, Next};
use axum::response::sse::{Event, KeepAlive, Sse};
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use directories_next::BaseDirs;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use tokio::sync::watch;
use tokio_stream::{StreamExt, wrappers::WatchStream};

use crate::app::{Action, Application, ProgramMotor, follower_motors, programming_ports};

include!(concat!(env!("OUT_DIR"), "/assets.rs"));

const COOKIE: &str = "so101_session";
const LOCAL_CSP: &str = "default-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; frame-ancestors 'none'";

#[derive(Clone, Serialize, Deserialize)]
pub struct ServiceAddress {
    pub port: u16,
    pub token: String,
}

impl ServiceAddress {
    pub fn url(&self) -> String {
        format!("http://127.0.0.1:{}", self.port)
    }
}

pub struct ServiceDirectory(PathBuf);

impl ServiceDirectory {
    pub fn locate() -> Result<Self> {
        let directories =
            BaseDirs::new().context("Could not locate the current user's directories")?;
        let directory = directories
            .runtime_dir()
            .unwrap_or_else(|| directories.data_local_dir())
            .join("hebbian-so101");
        Ok(Self(directory))
    }

    fn lock_file(&self) -> Result<std::fs::File> {
        std::fs::create_dir_all(&self.0)?;
        Ok(std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(self.0.join("commissioning.lock"))?)
    }

    pub fn lock(&self) -> Result<std::fs::File> {
        let lock = self.lock_file()?;
        lock.try_lock()
            .context("Another so101 setup or maintenance command is running")?;
        Ok(lock)
    }

    fn endpoint(&self) -> PathBuf {
        self.0.join("service.json")
    }

    pub fn running_client(&self) -> Result<Option<Client>> {
        let lock = self.lock_file()?;
        match lock.try_lock() {
            Ok(()) => Ok(None),
            Err(std::fs::TryLockError::WouldBlock) => self.address()?
                .map(Client::new).map(Some)
                .context("The setup service or a maintenance command is starting. Try again in a moment."),
            Err(error) => Err(error.into()),
        }
    }

    pub fn address(&self) -> Result<Option<ServiceAddress>> {
        match std::fs::read(self.endpoint()) {
            Ok(bytes) => Ok(Some(
                serde_json::from_slice(&bytes).context("Invalid local service address")?,
            )),
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(None),
            Err(error) => Err(error.into()),
        }
    }

    fn publish(&self, address: &ServiceAddress) -> Result<PublishedEndpoint> {
        let mut file = tempfile::NamedTempFile::new_in(&self.0)?;
        file.write_all(&serde_json::to_vec(address)?)?;
        let path = self.endpoint();
        file.persist(&path)?;
        Ok(PublishedEndpoint(path))
    }
}

struct PublishedEndpoint(PathBuf);

impl Drop for PublishedEndpoint {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}

#[derive(Clone)]
struct HttpState {
    address: ServiceAddress,
    app: Arc<Mutex<Application>>,
    updates: watch::Receiver<Update>,
}

type HttpError = (StatusCode, Json<Value>);
type HttpResult<T> = Result<T, HttpError>;

#[derive(Clone)]
struct Update {
    event: &'static str,
    payload: String,
}

#[derive(Deserialize, Serialize)]
pub struct ResetRequest {
    pub serial: String,
    pub motor_id: Option<u8>,
    pub confirmed: bool,
}

pub async fn serve(
    application: Application,
    directory: &ServiceDirectory,
    port: u16,
    no_browser: bool,
) -> Result<()> {
    let address = SocketAddrV4::new(Ipv4Addr::LOCALHOST, port);
    let listener = tokio::net::TcpListener::bind(address).await?;
    let address = ServiceAddress {
        port: listener.local_addr()?.port(),
        token: uuid::Uuid::new_v4().to_string(),
    };
    let application = Arc::new(Mutex::new(application));
    let (publish, updates) = watch::channel(Update {
        event: "view-error",
        payload: json!({"message": "Connecting to setup service"}).to_string(),
    });
    let sampling = tokio::spawn(sample(Arc::clone(&application), publish));
    let state = HttpState {
        address: address.clone(),
        app: application,
        updates,
    };
    let router = Router::new()
        .route("/api/action", post(action))
        .route("/api/events", get(events))
        .route("/api/state", get(view))
        .route("/api/controllers", get(controllers))
        .route("/api/motor-ids", get(motor_id_options).post(program_motor))
        .route("/api/reset", post(reset))
        .fallback(get(asset))
        .layer(DefaultBodyLimit::max(4096))
        .layer(middleware::from_fn_with_state(
            state.clone(),
            local_requests,
        ))
        .with_state(state);
    let _endpoint = directory.publish(&address)?;
    println!("SO-101 setup: {}/", address.url());
    if !no_browser && let Err(error) = open::that_detached(address.url()) {
        eprintln!("Could not open the browser: {error}");
    }
    let outcome = tokio::select! {
        result = axum::serve(listener, router) => result,
        result = tokio::signal::ctrl_c() => result,
    };
    sampling.abort();
    outcome?;
    Ok(())
}

async fn sample(app: Arc<Mutex<Application>>, sender: watch::Sender<Update>) {
    loop {
        let app = Arc::clone(&app);
        let update = tokio::task::spawn_blocking(move || {
            let mut app = app
                .lock()
                .map_err(|_| anyhow::anyhow!("Setup service lock failed"))?;
            app.poll();
            Ok::<_, anyhow::Error>(serde_json::to_string(&app.view()?)?)
        })
        .await;
        let update = match update {
            Ok(Ok(payload)) => Update {
                event: "view",
                payload,
            },
            Ok(Err(error)) => Update {
                event: "view-error",
                payload: json!({"message": format!("{error:#}")}).to_string(),
            },
            Err(_) => break,
        };
        sender.send_replace(update);
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
}

async fn local_requests(State(state): State<HttpState>, request: Request, next: Next) -> Response {
    let headers = request.headers();
    let hostname = format!("127.0.0.1:{}", state.address.port);
    if headers
        .get(header::HOST)
        .and_then(|value| value.to_str().ok())
        != Some(&hostname)
    {
        return (
            StatusCode::FORBIDDEN,
            Json(json!({ "message": "Use the local setup URL." })),
        )
            .into_response();
    }
    if request.uri().path().starts_with("/api/")
        && !authorized(headers, request.method(), &state.address)
    {
        return (
            StatusCode::UNAUTHORIZED,
            Json(json!({ "message": "Open the setup page or rerun the CLI command." })),
        )
            .into_response();
    }
    let mut response = next.run(request).await;
    let headers = response.headers_mut();
    headers.insert(header::CACHE_CONTROL, HeaderValue::from_static("no-store"));
    headers.insert(
        header::X_CONTENT_TYPE_OPTIONS,
        HeaderValue::from_static("nosniff"),
    );
    headers.insert(
        header::CONTENT_SECURITY_POLICY,
        HeaderValue::from_static(LOCAL_CSP),
    );
    response
}

fn authorized(headers: &HeaderMap, method: &Method, address: &ServiceAddress) -> bool {
    let bearer = format!("Bearer {}", address.token);
    if headers
        .get(header::AUTHORIZATION)
        .and_then(|value| value.to_str().ok())
        == Some(&bearer)
    {
        return true;
    }
    if method != Method::GET
        && headers
            .get(header::ORIGIN)
            .and_then(|value| value.to_str().ok())
            != Some(&address.url())
    {
        return false;
    }
    let cookie = format!("{COOKIE}={}", address.token);
    headers
        .get(header::COOKIE)
        .and_then(|value| value.to_str().ok())
        .is_some_and(|value| value.split(';').any(|part| part.trim() == cookie))
}

async fn asset(State(state): State<HttpState>, request: Request) -> Response {
    let path = request.uri().path().trim_start_matches('/');
    let index = matches!(path, "" | "motor-ids");
    let name = if index { "index.html" } else { path };
    let Some((_, bytes)) = ASSETS.iter().find(|(key, _)| *key == name) else {
        return StatusCode::NOT_FOUND.into_response();
    };
    let mut response = Response::new(Body::from(*bytes));
    if let Ok(mime) =
        HeaderValue::from_str(mime_guess::from_path(name).first_or_octet_stream().as_ref())
    {
        response.headers_mut().insert(header::CONTENT_TYPE, mime);
    }
    if index
        && let Ok(value) = HeaderValue::from_str(&format!(
            "{COOKIE}={}; HttpOnly; SameSite=Strict; Path=/",
            state.address.token
        ))
    {
        response.headers_mut().insert(header::SET_COOKIE, value);
    }
    response
}

async fn perform<T: Serialize + Send + 'static>(
    state: HttpState,
    operation: impl FnOnce(&mut Application) -> Result<T> + Send + 'static,
) -> HttpResult<Json<T>> {
    let result = tokio::task::spawn_blocking(move || {
        let mut app = state
            .app
            .lock()
            .map_err(|_| anyhow::anyhow!("Setup service lock failed"))?;
        operation(&mut app)
    })
    .await
    .map_err(bad_request)?;
    result.map(Json).map_err(bad_request)
}

fn bad_request(error: impl std::fmt::Display) -> HttpError {
    (
        StatusCode::BAD_REQUEST,
        Json(json!({ "message": format!("{error:#}") })),
    )
}

async fn action(
    State(state): State<HttpState>,
    Json(action): Json<Action>,
) -> HttpResult<impl IntoResponse> {
    perform(state, move |app| app.act(action)).await
}

async fn view(State(state): State<HttpState>) -> HttpResult<impl IntoResponse> {
    perform(state, |app| app.view()).await
}

async fn controllers(State(state): State<HttpState>) -> HttpResult<impl IntoResponse> {
    perform(state, |app| app.controllers()).await
}

async fn motor_id_options() -> HttpResult<Json<Value>> {
    let ports = tokio::task::spawn_blocking(programming_ports)
        .await
        .map_err(bad_request)?
        .map_err(bad_request)?;
    Ok(Json(json!({ "motors": follower_motors(), "ports": ports })))
}

async fn program_motor(
    State(state): State<HttpState>,
    Json(request): Json<ProgramMotor>,
) -> HttpResult<impl IntoResponse> {
    perform(state, move |app| {
        Ok(json!({ "motor_id": app.program(&request)? }))
    })
    .await
}

async fn reset(
    State(state): State<HttpState>,
    Json(request): Json<ResetRequest>,
) -> HttpResult<impl IntoResponse> {
    perform(state, move |app| Ok(json!({ "backups": app.standalone_reset(&request.serial, request.motor_id, request.confirmed)? }))).await
}

async fn events(State(state): State<HttpState>) -> impl IntoResponse {
    let stream = WatchStream::new(state.updates).map(|update| {
        Ok::<_, Infallible>(Event::default().event(update.event).data(update.payload))
    });
    Sse::new(stream).keep_alive(KeepAlive::default())
}

pub struct Client {
    agent: ureq::Agent,
    address: ServiceAddress,
}

impl Client {
    pub fn connect(directory: &ServiceDirectory) -> Result<Self> {
        directory
            .running_client()?
            .context("Start so101 setup --no-browser in another terminal")
    }

    pub fn new(address: ServiceAddress) -> Self {
        let agent = ureq::Agent::config_builder()
            .http_status_as_error(false)
            .proxy(None)
            .max_redirects(0)
            .timeout_connect(Some(Duration::from_secs(2)))
            .timeout_global(Some(Duration::from_secs(90)))
            .build()
            .into();
        Self { agent, address }
    }

    pub fn get(&self, path: &str) -> Result<Value> {
        let response = self
            .agent
            .get(format!("{}{path}", self.address.url()))
            .header(
                header::AUTHORIZATION,
                format!("Bearer {}", self.address.token),
            )
            .call()
            .context("Cannot reach the local setup service. Start so101 setup --no-browser.")?;
        Self::response(response)
    }

    pub fn post(&self, path: &str, request: &impl Serialize) -> Result<Value> {
        let response = self
            .agent
            .post(format!("{}{path}", self.address.url()))
            .header(
                header::AUTHORIZATION,
                format!("Bearer {}", self.address.token),
            )
            .send_json(request)
            .context("Cannot reach the local setup service. Start so101 setup --no-browser.")?;
        Self::response(response)
    }

    fn response(mut response: ureq::http::Response<ureq::Body>) -> Result<Value> {
        let status = response.status();
        let payload: Value = response
            .body_mut()
            .read_json()
            .with_context(|| format!("Invalid setup service response ({status})"))?;
        ensure!(
            status.is_success(),
            "{}",
            payload["message"]
                .as_str()
                .unwrap_or("Setup request rejected")
        );
        Ok(payload)
    }

    pub fn act(&self, action: &Action) -> Result<Value> {
        self.post("/api/action", action)
    }
}

pub fn default_setup_directory() -> Result<PathBuf> {
    if let Some(directory) =
        std::env::var_os("SO101_SETUP_DIRECTORY").filter(|value| !value.is_empty())
    {
        return Ok(PathBuf::from(directory));
    }
    let directories =
        BaseDirs::new().context("Could not locate the current user's config directory")?;
    Ok(directories.config_dir().join("so101-setup"))
}

pub fn resolve_directory(path: Option<&Path>) -> Result<PathBuf> {
    path.map_or_else(default_setup_directory, |path| Ok(path.to_path_buf()))
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn bundled_ui_and_cli_share_a_local_server() -> Result<()> {
        let scratch = tempfile::tempdir()?;
        let directory = ServiceDirectory(scratch.path().to_owned());
        let app = Application::new(
            scratch.path().join("setup"),
            scratch.path().join("lerobot"),
            hebbian_so101::setup::ModelVariant::SoftGripperCamera,
            false,
        )?;
        let server_directory = ServiceDirectory(directory.0.clone());
        let server = tokio::spawn(async move {
            let _lock = server_directory.lock()?;
            serve(app, &server_directory, 0, true).await
        });
        let result = async {
            let address = tokio::time::timeout(Duration::from_secs(5), async {
                loop {
                    if let Some(address) = directory.address()? {
                        return Ok::<_, anyhow::Error>(address);
                    }
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            })
            .await??;
            let client = Client::connect(&directory)?;
            tokio::task::spawn_blocking(move || {
                let browser = ureq::Agent::config_builder()
                    .proxy(None)
                    .http_status_as_error(false)
                    .timeout_global(Some(Duration::from_secs(5)))
                    .build()
                    .new_agent();
                let mut page = browser.get(format!("{}/", address.url())).call()?;
                assert_eq!(page.status(), StatusCode::OK);
                let cookie_header = page
                    .headers()
                    .get(header::SET_COOKIE)
                    .context("No browser session")?
                    .to_str()?;
                let cookie = cookie_header
                    .split(';')
                    .next()
                    .context("Invalid cookie")?
                    .to_owned();
                let document = page.body_mut().read_to_string()?;
                assert!(document.contains("<title>Hebbian Robotics SO-101</title>"));
                let model = browser
                    .get(format!("{}/robot/so101/robot.urdf", address.url()))
                    .call()?;
                assert_eq!(model.status(), StatusCode::OK);
                let rejected = browser.get(format!("{}/api/state", address.url())).call()?;
                assert_eq!(rejected.status(), StatusCode::UNAUTHORIZED);

                let mut browser_state = browser
                    .get(format!("{}/api/state", address.url()))
                    .header(header::COOKIE, &cookie)
                    .call()?;
                assert_eq!(browser_state.status(), StatusCode::OK);
                let browser_state: Value = browser_state.body_mut().read_json()?;
                let terminal_state = client.get("/api/state")?;
                assert_eq!(terminal_state["stage"], "disconnected");
                assert_eq!(terminal_state["stage"], browser_state["stage"]);

                let rejected_save = client.act(&Action::Save {
                    model_verified: true,
                    review_revision: 0,
                });
                let error = rejected_save
                    .err()
                    .context("The service accepted a save without an arm")?;
                assert!(error.to_string().contains("Connect an arm first"));
                Ok::<_, anyhow::Error>(())
            })
            .await?
        }
        .await;
        server.abort();
        let _ = server.await;
        result
    }

    // Another local account can open the published loopback port. The setup page must not
    // turn that visitor into a session that can reset servos, program motor IDs, or change
    // calibration. The operator's launch token still has to work.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn loopback_page_visitor_cannot_drive_setup_actions() -> Result<()> {
        let scratch = tempfile::tempdir()?;
        let setup_root = scratch.path().join("setup");
        let calibration_root = scratch.path().join("lerobot");
        let directory = ServiceDirectory(scratch.path().to_owned());
        let app = Application::new(
            setup_root.clone(),
            calibration_root.clone(),
            hebbian_so101::setup::ModelVariant::SoftGripperCamera,
            false,
        )?;
        let server_directory = ServiceDirectory(directory.0.clone());
        let server = tokio::spawn(async move {
            let _lock = server_directory.lock()?;
            serve(app, &server_directory, 0, true).await
        });
        let result = async {
            let address = tokio::time::timeout(Duration::from_secs(5), async {
                loop {
                    if let Some(address) = directory.address()? {
                        return Ok::<_, anyhow::Error>(address);
                    }
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            })
            .await??;
            let port = address.port;
            let operator = Client::new(address);
            tokio::task::spawn_blocking(move || {
                reject_port_only_visitor(&operator, port, &setup_root, &calibration_root)?;
                operator.act(&Action::Disconnect)?;
                Ok::<_, anyhow::Error>(())
            })
            .await?
        }
        .await;
        server.abort();
        let _ = server.await;
        result
    }

    fn reject_port_only_visitor(
        operator: &Client,
        port: u16,
        setup_root: &Path,
        calibration_root: &Path,
    ) -> Result<()> {
        let before_view = operator.get("/api/state")?;
        let before_setup = directory_entries(setup_root)?;
        let before_calibration = directory_entries(calibration_root)?;
        let agent = ureq::Agent::config_builder()
            .proxy(None)
            .http_status_as_error(false)
            .timeout_global(Some(Duration::from_secs(5)))
            .build()
            .new_agent();
        let mut cookies = Vec::new();
        for path in ["/", "/motor-ids"] {
            let (status, cookie, body) = fetch_page(&agent, port, path)?;
            if status == StatusCode::FORBIDDEN && body.contains("Use the local setup URL") {
                anyhow::bail!("visitor Host header did not select the setup listener");
            }
            if let Some(cookie) = cookie {
                cookies.push(cookie);
            }
        }

        let mut failures = Vec::new();
        for cookie in std::iter::once(None).chain(cookies.iter().map(Some)) {
            let presentation = if cookie.is_some() {
                "setup-page cookie"
            } else {
                "no cookie"
            };
            for probe in privileged_probes() {
                let (status, body) = send_probe(&agent, port, &probe, cookie)?;
                if status == StatusCode::UNAUTHORIZED || status == StatusCode::FORBIDDEN {
                    continue;
                }
                failures.push(format!(
                    "{} {} with {presentation} -> {status} {}",
                    probe.method,
                    probe.path,
                    response_message(&body)
                ));
            }
        }

        if directory_entries(setup_root)? != before_setup
            || directory_entries(calibration_root)? != before_calibration
        {
            failures.push(
                "setup or calibration files changed after the unauthenticated requests".to_owned(),
            );
        }
        if operator.get("/api/state")? != before_view {
            failures.push("the setup view changed after the unauthenticated requests".to_owned());
        }
        anyhow::ensure!(
            failures.is_empty(),
            "a client that only knows the setup port reached privileged operations:\n{}",
            failures.join("\n")
        );
        Ok(())
    }

    struct Probe {
        method: &'static str,
        path: &'static str,
        body: Option<Value>,
    }

    fn privileged_probes() -> Vec<Probe> {
        let action = |value: Value| Probe {
            method: "POST",
            path: "/api/action",
            body: Some(value),
        };
        vec![
            Probe {
                method: "GET",
                path: "/api/state",
                body: None,
            },
            Probe {
                method: "GET",
                path: "/api/events",
                body: None,
            },
            Probe {
                method: "GET",
                path: "/api/controllers",
                body: None,
            },
            Probe {
                method: "GET",
                path: "/api/motor-ids",
                body: None,
            },
            Probe {
                method: "POST",
                path: "/api/motor-ids",
                body: Some(json!({
                    "port": "/dev/invalid-so101-port",
                    "motor_id": 1,
                    "isolated_motor": true
                })),
            },
            Probe {
                method: "POST",
                path: "/api/reset",
                body: Some(json!({
                    "serial": "ABC",
                    "motor_id": 1,
                    "confirmed": true
                })),
            },
            Probe {
                method: "POST",
                path: "/api/reset",
                body: Some(json!({
                    "serial": "ABC",
                    "motor_id": null,
                    "confirmed": true
                })),
            },
            action(json!({"action": "connect", "serial": "ABC"})),
            action(json!({"action": "begin", "serial": "ABC", "skip_reset": true})),
            action(json!({"action": "disconnect"})),
            action(json!({"action": "reset_servos"})),
            action(json!({"action": "skip_servo_reset"})),
            action(json!({"action": "restart"})),
            action(json!({"action": "reopen"})),
            action(json!({"action": "start_sweep", "joint_index": null})),
            action(json!({"action": "finish_sweep"})),
            action(json!({
                "action": "map_motor_ids",
                "motor_ids": {
                    "shoulder_pan": 1,
                    "shoulder_lift": 2,
                    "elbow_flex": 3,
                    "wrist_flex": 4,
                    "wrist_roll": 5,
                    "gripper": 6
                }
            })),
            action(json!({"action": "check_motor_order"})),
            action(json!({"action": "apply_detected_motor_order"})),
            action(json!({"action": "reverse_joint", "index": 0})),
            action(json!({"action": "adjust_joint", "index": 0, "delta_radians": 0.1})),
            action(json!({"action": "save", "model_verified": true, "review_revision": 0})),
        ]
    }

    fn fetch_page(
        agent: &ureq::Agent,
        port: u16,
        path: &str,
    ) -> Result<(StatusCode, Option<String>, String)> {
        let mut response = agent
            .get(format!("http://127.0.0.1:{port}{path}"))
            .header(header::HOST, format!("127.0.0.1:{port}"))
            .call()?;
        let status = response.status();
        let cookie = response
            .headers()
            .get(header::SET_COOKIE)
            .and_then(|value| value.to_str().ok())
            .and_then(|value| value.split(';').next())
            .map(str::trim)
            .filter(|value| !value.is_empty())
            .map(ToOwned::to_owned);
        let body = response.body_mut().read_to_string()?;
        Ok((status, cookie, body))
    }

    fn send_probe(
        agent: &ureq::Agent,
        port: u16,
        probe: &Probe,
        cookie: Option<&String>,
    ) -> Result<(StatusCode, String)> {
        let url = format!("http://127.0.0.1:{port}{}", probe.path);
        let host = format!("127.0.0.1:{port}");
        let origin = format!("http://127.0.0.1:{port}");
        let response = if probe.method == "GET" {
            let request = agent
                .get(&url)
                .header(header::HOST, &host)
                .header(header::ORIGIN, &origin);
            let request = if let Some(cookie) = cookie {
                request.header(header::COOKIE, cookie)
            } else {
                request
            };
            request.call()?
        } else {
            let request = agent
                .post(&url)
                .header(header::HOST, &host)
                .header(header::ORIGIN, &origin);
            let request = if let Some(cookie) = cookie {
                request.header(header::COOKIE, cookie)
            } else {
                request
            };
            if let Some(body) = &probe.body {
                request.send_json(body)?
            } else {
                request.send_empty()?
            }
        };
        Ok(response_parts(response))
    }

    fn response_parts(mut response: ureq::http::Response<ureq::Body>) -> (StatusCode, String) {
        let status = response.status();
        let event_stream = response
            .headers()
            .get(header::CONTENT_TYPE)
            .and_then(|value| value.to_str().ok())
            .is_some_and(|value| value.contains("text/event-stream"));
        if event_stream {
            return (status, String::from("event stream"));
        }
        let body = response
            .body_mut()
            .read_to_string()
            .unwrap_or_else(|error| error.to_string());
        (status, body)
    }

    fn response_message(body: &str) -> String {
        serde_json::from_str::<Value>(body)
            .ok()
            .and_then(|value| {
                value
                    .get("message")
                    .and_then(Value::as_str)
                    .map(ToOwned::to_owned)
            })
            .unwrap_or_else(|| format!("{}-byte body", body.len()))
    }

    fn directory_entries(root: &Path) -> Result<BTreeMap<PathBuf, DirectoryEntry>> {
        let mut entries = BTreeMap::new();
        if root.exists() {
            collect_entries(root, root, &mut entries)?;
        }
        Ok(entries)
    }

    fn collect_entries(
        root: &Path,
        directory: &Path,
        entries: &mut BTreeMap<PathBuf, DirectoryEntry>,
    ) -> Result<()> {
        let relative = directory
            .strip_prefix(root)
            .unwrap_or(directory)
            .to_path_buf();
        if relative.components().next().is_some() {
            entries.insert(relative, DirectoryEntry::Directory);
        }
        for entry in std::fs::read_dir(directory)? {
            let path = entry?.path();
            if path.is_dir() {
                collect_entries(root, &path, entries)?;
            } else {
                let relative = path.strip_prefix(root).unwrap_or(&path).to_path_buf();
                entries.insert(relative, DirectoryEntry::File(std::fs::read(&path)?));
            }
        }
        Ok(())
    }

    #[derive(PartialEq, Eq)]
    enum DirectoryEntry {
        Directory,
        File(Vec<u8>),
    }
}
