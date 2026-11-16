import { useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import { z } from "zod";
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
  reference_joints_radians: z.array(z.number().finite()).length(5),
  reference_gripper_radians: z.number().finite(),
  message: z.string(),
  command: z.string(),
});
type CalibrationView = z.infer<typeof viewSchema>;
type Action =
  | { action: "connect"; serial: string }
  | { action: "disconnect" | "capture_reference" | "finish_sweep" | "restart" }
  | { action: "reverse_joint"; index: number }
  | { action: "capture_gripper_endpoint"; endpoint: "closed" | "open" }
  | { action: "save"; model_verified: true };
const jointNames = [
  "shoulder_pan",
  "shoulder_lift",
  "elbow_flex",
  "wrist_flex",
  "wrist_roll",
];
const referenceJoints = [0, 0, 0, 0, 0];
const fallbackReferenceProjection = {
  arm: "follower",
  joints: referenceJoints,
  opening: 0.55,
  withinTolerance: true,
  stale: false,
};
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
const token = window.location.hash.slice(1);

async function requestView(
  action?: Action,
  signal?: AbortSignal,
): Promise<CalibrationView> {
  const response = await fetch(action ? "api/action" : "api/state", {
    method: action ? "POST" : "GET",
    headers: {
      Authorization: `Bearer ${token}`,
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

function App() {
  const [view, setView] = useState<CalibrationView | null>(null);
  const [connectionError, setConnectionError] = useState("");
  const [actionError, setActionError] = useState("");
  const [busy, setBusy] = useState(false);
  const [verified, setVerified] = useState(false);
  const [showReference, setShowReference] = useState(true);
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
      if (action.action === "capture_reference") setShowReference(false);
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
  const referenceProjection = view
    ? {
        ...fallbackReferenceProjection,
        joints: view.reference_joints_radians,
        opening: (view.reference_gripper_radians + 0.174533) / 1.674533,
      }
    : fallbackReferenceProjection;
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
      <header className="page-heading">
        <div>
          <h1>SO-101 calibration</h1>
          <p>Calibrate and align this arm.</p>
        </div>
        <span className="connection">
          {view?.serial || "Waiting for an arm"}
        </span>
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
          {stage === "reference" ? (
            <>
              <h2>Match the reference pose</h2>
              {view?.message ? (
                <p className="detail" role="status">
                  {view.message}
                </p>
              ) : null}
              <p>
                Support the arm and place it in the translucent blue pose. Match
                the wrist orientation and set the gripper to the illustrated
                opening.
              </p>
              <p>
                If it is already in this position, capture it now. The reference
                stays fixed while the gray model follows the current readings.
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
          {stage === "sweep" ? (
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
                wrist orientation, and gripper. Existing calibration is reused
                when it matches the motors.
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
              <h2>{view?.command ? "Ready for the viewer" : "Setup saved"}</h2>
              <p>
                The serial port is released. Calibration and model alignment are
                saved in your selected setup directory.
              </p>
              {view?.command ? <pre>{view.command}</pre> : null}
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
                    setShowReference(true);
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
          <RobotScene
            modelUrl={
              view?.model_variant === "soft-gripper-camera"
                ? "robot/so101/robot-soft-gripper-camera.urdf"
                : "robot/so101/robot.urdf"
            }
            cameraPosition={calibrationCameraPosition}
            cameraTarget={calibrationCameraTarget}
            jointNames={jointNames}
            measuredJoints={sceneArms[0]?.measuredJoints ?? []}
            gripperAngleRadians={sceneArms[0]?.gripperAngleRadians}
            referenceJoints={showReference ? referenceProjection.joints : null}
            referenceOpening={referenceProjection.opening}
          />
          <label className="reference-toggle">
            <input
              type="checkbox"
              checked={showReference}
              onChange={(event) => setShowReference(event.target.checked)}
            />
            Show reference pose
          </label>
          <p className="legend">
            <span className="measured-key" />
            Measured arm <span className="reference-key" />
            Reference pose
          </p>
          {view?.joints.length ? (
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
                    <th>Angle</th>
                    <th>Travel recorded</th>
                    {stage === "sweep" ? <th>Direction</th> : null}
                  </tr>
                </thead>
                <tbody>
                  {view.joints.map((joint, index) => (
                    <tr key={joint.name}>
                      <th scope="row">{labels[joint.name] ?? joint.name}</th>
                      <td>
                        {((joint.position_radians * 180) / Math.PI).toFixed(1)}°
                      </td>
                      <td>
                        {joint.name === "wrist_roll"
                          ? "Full turn (LeRobot)"
                          : joint.range_max > joint.range_min
                            ? `${joint.range_min}–${joint.range_max}`
                            : "Move this joint"}
                      </td>
                      {stage === "sweep" ? (
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
