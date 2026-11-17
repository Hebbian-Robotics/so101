import { useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import { z } from "zod";
import hebbianLogo from "./hebbian-logo.svg";
import { RobotScene } from "./robot-scene.js";
import "./style.css";

const jointSchema = z.object({
  name: z.string(),
  position_radians: z.number().finite(),
  range_min: z.number().int(),
  range_max: z.number().int(),
  sign: z.union([z.literal(-1), z.literal(1)]),
});
const viewSchema = z.object({
  model_variant: z.enum(["stock", "soft-gripper-camera"]),
  stage: z.enum([
    "disconnected",
    "reference",
    "sweep",
    "review",
    "saved",
    "error",
  ]),
  controllers: z.array(
    z.object({ serial: z.string(), port: z.string(), registered: z.boolean() }),
  ),
  serial: z.string(),
  joints: z.array(jointSchema),
  gripper_closed_recorded: z.boolean(),
  gripper_open_recorded: z.boolean(),
  gripper_opening: z.number().finite(),
  estimated: z.boolean().default(false),
  sweep_only: z.boolean().default(false),
  home_joints_radians: z.array(z.number().finite()).length(5),
  home_gripper_radians: z.number().finite(),
  reference_joints_radians: z.array(z.number().finite()).length(5),
  reference_gripper_radians: z.number().finite(),
  message: z.string(),
  configuration_path: z.string().default(""),
  calibration_path: z.string().default(""),
});
type CalibrationView = z.infer<typeof viewSchema>;
type Action =
  | { action: "connect"; serial: string }
  | {
      action:
        | "disconnect"
        | "capture_reference"
        | "start_sweep"
        | "finish_sweep"
        | "restart"
        | "reopen";
    }
  | { action: "reverse_joint"; index: number }
  | { action: "adjust_joint"; index: number; delta_radians: number }
  | { action: "capture_gripper_endpoint"; endpoint: "closed" | "open" }
  | { action: "save"; model_verified: true };
const jointNames = [
  "shoulder_pan",
  "shoulder_lift",
  "elbow_flex",
  "wrist_flex",
  "wrist_roll",
];
const labels: Record<string, string> = {
  shoulder_pan: "Shoulder pan",
  shoulder_lift: "Shoulder lift",
  elbow_flex: "Elbow",
  wrist_flex: "Wrist pitch",
  wrist_roll: "Wrist roll",
  gripper: "Gripper",
};
const calibrationCameraPosition: [number, number, number] = [0.9, -0.9, 0.75];
const calibrationCameraTarget: [number, number, number] = [0.1, 0, 0.25];

async function requestView(
  action?: Action,
  signal?: AbortSignal,
): Promise<CalibrationView> {
  const response = await fetch(action ? "api/action" : "api/state", {
    method: action ? "POST" : "GET",
    headers: {
      ...(action ? { "Content-Type": "application/json" } : {}),
    },
    ...(action ? { body: JSON.stringify(action) } : {}),
    ...(signal ? { signal } : {}),
  });
  const payload: unknown = await response.json();
  if (!response.ok) {
    const error = z.object({ message: z.string() }).safeParse(payload);
    throw new Error(
      error.success
        ? error.data.message
        : `Calibration request failed (${response.status}).`,
    );
  }
  return viewSchema.parse(payload);
}

type ThemePreference = "system" | "light" | "dark";
const themeStorageKey = "so101-calibration-theme";

function readThemePreference(): ThemePreference {
  try {
    const stored = localStorage.getItem(themeStorageKey);
    return stored === "light" || stored === "dark" ? stored : "system";
  } catch {
    return "system";
  }
}

function App() {
  const [theme, setTheme] = useState<ThemePreference>(readThemePreference);
  const [systemDark, setSystemDark] = useState(
    () => window.matchMedia("(prefers-color-scheme: dark)").matches,
  );
  useEffect(() => {
    if (theme === "system")
      document.documentElement.removeAttribute("data-theme");
    else document.documentElement.setAttribute("data-theme", theme);
    try {
      if (theme === "system") localStorage.removeItem(themeStorageKey);
      else localStorage.setItem(themeStorageKey, theme);
    } catch {
      // The theme still works when site storage is unavailable.
    }
  }, [theme]);
  useEffect(() => {
    const preference = window.matchMedia("(prefers-color-scheme: dark)");
    const updateSystemTheme = () => setSystemDark(preference.matches);
    const syncStoredTheme = (event: StorageEvent) => {
      if (event.key === themeStorageKey) setTheme(readThemePreference());
    };
    preference.addEventListener("change", updateSystemTheme);
    window.addEventListener("storage", syncStoredTheme);
    return () => {
      preference.removeEventListener("change", updateSystemTheme);
      window.removeEventListener("storage", syncStoredTheme);
    };
  }, []);
  const darkMode = theme === "dark" || (theme === "system" && systemDark);
  const [view, setView] = useState<CalibrationView | null>(null);
  const [connectionError, setConnectionError] = useState("");
  const [actionError, setActionError] = useState("");
  const [busy, setBusy] = useState(false);
  const [verified, setVerified] = useState(false);
  const generation = useRef(0);
  const acting = useRef(false);
  useEffect(() => {
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    async function poll() {
      const currentGeneration = generation.current;
      try {
        if (!acting.current) {
          const fresh = await requestView(undefined, controller.signal);
          if (currentGeneration === generation.current) {
            setView(fresh);
            setConnectionError("");
          }
        }
      } catch (error) {
        if (
          !controller.signal.aborted &&
          currentGeneration === generation.current
        )
          setConnectionError(
            error instanceof Error ? error.message : String(error),
          );
      } finally {
        if (!controller.signal.aborted) timer = setTimeout(poll, 200);
      }
    }
    void poll();
    return () => {
      controller.abort();
      clearTimeout(timer);
    };
  }, []);
  async function perform(action: Action) {
    if (acting.current) return;
    acting.current = true;
    generation.current += 1;
    setBusy(true);
    setVerified(false);
    setActionError("");
    try {
      setView(await requestView(action));
      setConnectionError("");
    } catch (error) {
      setActionError(error instanceof Error ? error.message : String(error));
    } finally {
      acting.current = false;
      setBusy(false);
    }
  }
  const stage = view?.stage ?? "disconnected";
  const connected = Boolean(view?.serial);
  const disabled = busy || Boolean(connectionError);
  const sceneArms = [
    {
      arm: "follower",
      measuredJoints: view?.joints.length
        ? view.joints.slice(0, 5).map((joint) => joint.position_radians)
        : [null, null, null, null, null],
      gripperOpening: view?.gripper_opening ?? 0.55,
      gripperAngleRadians: view?.joints[5]?.position_radians,
    },
  ];
  return (
    <main>
      <div className="brand">
        <img src={hebbianLogo} alt="" />
        <span>
          Hebbian <strong>Robot Use</strong>
        </span>
      </div>
      <header className="page-heading">
        <div>
          <h1>SO-101 calibration</h1>
          <p>Calibrate and align this arm.</p>
        </div>
        <div className="heading-controls">
          <span className="connection">
            {view?.serial || "Waiting for an arm"}
          </span>
          <label className="theme-control">
            Theme
            <select
              value={theme}
              onChange={(event) =>
                setTheme(event.target.value as ThemePreference)
              }
            >
              <option value="system">System</option>
              <option value="light">Light</option>
              <option value="dark">Dark</option>
            </select>
          </label>
        </div>
      </header>
      <div className="workspace">
        <section className="instructions" aria-label="Calibration steps">
          <ol className="steps" aria-label="Progress">
            {["Connect", "Match pose", "Record travel", "Review"].map(
              (label, index) => (
                <li
                  key={label}
                  aria-current={
                    index ===
                    {
                      disconnected: 0,
                      reference: 1,
                      sweep: 2,
                      review: 3,
                      saved: 3,
                      error: 3,
                    }[stage]
                      ? "step"
                      : undefined
                  }
                >
                  {label}
                </li>
              ),
            )}
          </ol>
          {connectionError ? (
            <p role="alert" className="error">
              {connectionError}
            </p>
          ) : null}
          {actionError ? (
            <p role="alert" className="error">
              {actionError}
            </p>
          ) : null}
          {stage === "disconnected" ? (
            <>
              <h2>Connect your arm</h2>
              <p>
                Plug in the USB controller and power the arm with torque off. A
                registered arm is selected automatically.
              </p>
              {view?.controllers.map((controller) => (
                <button
                  className="controller"
                  type="button"
                  key={`${controller.serial}:${controller.port}`}
                  disabled={disabled}
                  onClick={() =>
                    void perform({
                      action: "connect",
                      serial: controller.serial,
                    })
                  }
                >
                  <strong>{controller.serial}</strong>
                  <span>
                    {controller.registered
                      ? "Saved configuration"
                      : "Set up this controller"}{" "}
                    · {controller.port}
                  </span>
                </button>
              ))}
              <p className="detail">
                {view?.message || "Looking for USB controllers…"}
              </p>
            </>
          ) : null}
          {stage === "reference" &&
          view?.model_variant === "soft-gripper-camera" ? (
            <>
              <h2>Start calibration</h2>
              <p>
                Support and disable the arm. Place it in the resting pose shown,
                with the gripper fully closed. Start the sweep to capture this
                reference, then gently move each joint through its travel.
              </p>
              <button
                className="primary"
                type="button"
                disabled={disabled}
                onClick={() => void perform({ action: "start_sweep" })}
              >
                Start sweep
              </button>
            </>
          ) : null}
          {stage === "reference" &&
          view?.model_variant !== "soft-gripper-camera" ? (
            <>
              <h2>Match the reference pose</h2>
              {view?.message ? (
                <p className="detail" role="status">
                  {view.message}
                </p>
              ) : null}
              <p>
                Support the arm. Set each of the five joints to its model zero
                and the gripper to approximately 43° open.
              </p>
              <p>
                When the physical arm is positioned, capture it. The live model
                and joint readings appear after this step.
              </p>
              <button
                className="primary"
                type="button"
                disabled={disabled}
                onClick={() => void perform({ action: "capture_reference" })}
              >
                Use this position
              </button>
            </>
          ) : null}
          {stage === "sweep" && view?.sweep_only ? (
            <>
              <h2>Record safe travel</h2>
              <p>
                Support the arm. Slowly move each joint from one gentle stop to
                the other, including wrist rotation. Fully close and open the
                gripper without forcing it.
              </p>
              <p>
                Move slowly enough for the readings to follow. The model appears
                after you finish the sweep.
              </p>
              <button
                className="primary"
                type="button"
                disabled={disabled}
                onClick={() => void perform({ action: "finish_sweep" })}
              >
                Finish recording
              </button>
            </>
          ) : null}
          {stage === "sweep" && !view?.sweep_only ? (
            <>
              <h2>Record safe travel</h2>
              <p>
                Gently move each joint through its safe travel, one at a time.
                Close the gripper and record its closed position, then open it
                to the safe endpoint and record open. Keep supporting the arm.
              </p>
              <p>
                Check that the gray model follows the physical arm. If one joint
                moves in the wrong direction, reverse it in the table.
              </p>
              <div className="secondary-actions">
                <button
                  type="button"
                  disabled={disabled}
                  onClick={() =>
                    void perform({
                      action: "capture_gripper_endpoint",
                      endpoint: "closed",
                    })
                  }
                >
                  {view?.gripper_closed_recorded
                    ? "Closed position recorded"
                    : "Record closed gripper"}
                </button>
                <button
                  type="button"
                  disabled={disabled}
                  onClick={() =>
                    void perform({
                      action: "capture_gripper_endpoint",
                      endpoint: "open",
                    })
                  }
                >
                  {view?.gripper_open_recorded
                    ? "Open position recorded"
                    : "Record open gripper"}
                </button>
              </div>
              <button
                className="primary"
                type="button"
                disabled={disabled}
                onClick={() => void perform({ action: "finish_sweep" })}
              >
                Finish recording
              </button>
            </>
          ) : null}
          {stage === "review" ? (
            <>
              <h2>Check the live model</h2>
              <p>
                Move the supported arm by hand. Check the joint directions,
                wrist orientation, and gripper. Sweep-based alignment is an
                estimate; adjust any joint that does not match before saving.
              </p>
              <label className="verification">
                <input
                  type="checkbox"
                  checked={verified}
                  disabled={disabled}
                  onChange={(event) => setVerified(event.target.checked)}
                />
                The model follows this physical arm correctly.
              </label>
              <button
                className="primary"
                type="button"
                disabled={disabled || !verified}
                onClick={() =>
                  void perform({ action: "save", model_verified: true })
                }
              >
                {busy ? "Saving…" : "Save setup"}
              </button>
              <p className="detail">
                Save backs up existing files, verifies calibration and gain
                registers with torque off, and updates this arm’s setup files.
              </p>
            </>
          ) : null}
          {stage === "saved" ? (
            <>
              <h2>Setup saved</h2>
              <p>
                The serial port is released. Calibration and model alignment are
                saved in your selected setup directory.
              </p>
              <button
                type="button"
                disabled={disabled}
                onClick={() => void perform({ action: "reopen" })}
              >
                Adjust saved setup
              </button>
              <p>Saved files:</p>
              <pre>
                {[view?.configuration_path, view?.calibration_path]
                  .filter(Boolean)
                  .join("\n")}
              </pre>
              <p className="detail">
                If these files are in a shared configuration repository, review
                and share the changes with your team.
              </p>
              <p className="detail">{view?.message}</p>
            </>
          ) : null}
          {stage === "error" ? (
            <>
              <h2>Inspect the arm before continuing</h2>
              <p role="alert" className="error">
                {view?.message}
              </p>
            </>
          ) : null}
          {connected && stage !== "saved" && stage !== "error" ? (
            <div className="secondary-actions">
              {stage !== "reference" ? (
                <button
                  type="button"
                  disabled={disabled}
                  onClick={() => {
                    void perform({ action: "restart" });
                  }}
                >
                  Start new calibration
                </button>
              ) : null}
              <button
                type="button"
                disabled={busy}
                onClick={() => void perform({ action: "disconnect" })}
              >
                Discard and disconnect
              </button>
            </div>
          ) : null}
          <p className="detail">
            This tool never enables torque or commands motion. Use your control
            application for powered operation.
          </p>
        </section>
        <section className="visual-feedback" aria-label="Calibration feedback">
          {view?.sweep_only ? (
            <p role="status">The model appears after the sweep.</p>
          ) : (
            <RobotScene
              darkMode={darkMode}
              modelUrl={
                view?.model_variant === "soft-gripper-camera"
                  ? "robot/so101/robot-soft-gripper-camera.urdf"
                  : "robot/so101/robot.urdf"
              }
              cameraPosition={calibrationCameraPosition}
              cameraTarget={calibrationCameraTarget}
              jointNames={jointNames}
              measuredJoints={
                stage === "reference" &&
                view?.model_variant === "soft-gripper-camera"
                  ? view.home_joints_radians
                  : stage === "reference"
                    ? []
                    : (sceneArms[0]?.measuredJoints ?? [])
              }
              gripperAngleRadians={
                stage === "reference"
                  ? view?.home_gripper_radians
                  : sceneArms[0]?.gripperAngleRadians
              }
            />
          )}
          {stage === "reference" &&
          view?.model_variant === "soft-gripper-camera" ? (
            <p>Resting pose reference</p>
          ) : null}
          {stage !== "reference" && view?.joints.length ? (
            <div className="joint-readings">
              <table>
                <caption>
                  {stage === "saved"
                    ? "Last readings before saving"
                    : "Live joint readings and recorded encoder travel"}
                </caption>
                <thead>
                  <tr>
                    <th>Joint</th>
                    {!view?.sweep_only && <th>Angle</th>}
                    <th>Travel recorded</th>
                    {(stage === "sweep" && !view?.sweep_only) ||
                    stage === "review" ? (
                      <th>Adjust model</th>
                    ) : null}
                  </tr>
                </thead>
                <tbody>
                  {view.joints.map((joint, index) => (
                    <tr key={joint.name}>
                      <th scope="row">{labels[joint.name] ?? joint.name}</th>
                      {!view?.sweep_only && (
                        <td>
                          {((joint.position_radians * 180) / Math.PI).toFixed(
                            1,
                          )}
                          °
                        </td>
                      )}
                      <td>
                        {view?.sweep_only
                          ? joint.range_max > joint.range_min
                            ? `${(((joint.range_max - joint.range_min) * 360) / 4096).toFixed(0)}°`
                            : "Move this joint"
                          : joint.name === "wrist_roll" && !view?.estimated
                            ? "Full turn (LeRobot)"
                            : joint.range_max > joint.range_min
                              ? `${joint.range_min}–${joint.range_max}`
                              : "Move this joint"}
                      </td>
                      {(stage === "sweep" && !view?.sweep_only) ||
                      stage === "review" ? (
                        <td>
                          <button
                            type="button"
                            disabled={disabled}
                            aria-label={`Reverse ${labels[joint.name] ?? joint.name}`}
                            onClick={() =>
                              void perform({ action: "reverse_joint", index })
                            }
                          >
                            Reverse
                          </button>
                          {stage === "review" &&
                            [-1, 1].map((direction) => (
                              <button
                                key={direction}
                                type="button"
                                disabled={disabled}
                                aria-label={`Adjust ${labels[joint.name] ?? joint.name} ${direction} degree`}
                                onClick={() =>
                                  void perform({
                                    action: "adjust_joint",
                                    index,
                                    delta_radians: (direction * Math.PI) / 180,
                                  })
                                }
                              >
                                {direction > 0 ? "+1°" : "−1°"}
                              </button>
                            ))}
                        </td>
                      ) : null}
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          ) : null}
        </section>
      </div>
    </main>
  );
}

const root = document.getElementById("root");
if (!root) throw new Error("Calibration root element is missing");
createRoot(root).render(<App />);
