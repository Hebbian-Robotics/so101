import { useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import { z } from "zod";
import hebbianLogo from "./hebbian-logo.svg";
import { MotorIds } from "./motor-ids.js";
import { RobotScene } from "./robot-scene.js";
import "./style.css";

const jointSchema = z.object({
  name: z.string(),
  position_radians: z.number().finite(),
  range_min: z.number().int(),
  range_max: z.number().int(),
  moving: z.boolean().default(false),
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
  gripper_opening: z.number().finite(),
  estimated: z.boolean().default(false),
  can_recalibrate_joint: z.boolean(),
  offer_servo_reset: z.boolean().default(false),
  motor_ids: z.record(z.string(), z.number().int().min(1).max(6)).optional(),
  checking_motor_order: z.boolean().default(false),
  detected_base_motor: z
    .union([z.literal(1), z.literal(6)])
    .nullable()
    .default(null),
  servo_reset_incomplete: z.boolean().default(false),
  recalibrating_joint: z.number().int().min(0).max(5).nullable(),
  home_joints_radians: z.array(z.number().finite()).length(5),
  home_gripper_radians: z.number().finite(),
  message: z.string(),
  configuration_path: z.string().default(""),
  calibration_path: z.string().default(""),
});
type CalibrationView = z.infer<typeof viewSchema>;
type Action =
  | { action: "connect"; serial: string }
  | { action: "map_motor_ids"; motor_ids: Record<string, number> }
  | { action: "start_sweep"; joint_index?: number }
  | {
      action:
        | "disconnect"
        | "finish_sweep"
        | "check_motor_order"
        | "apply_detected_motor_order"
        | "reset_servos"
        | "skip_servo_reset"
        | "restart"
        | "reopen";
    }
  | { action: "reverse_joint"; index: number }
  | { action: "adjust_joint"; index: number; delta_radians: number }
  | { action: "save"; model_verified: true };
const jointNames = [
  "shoulder_pan",
  "shoulder_lift",
  "elbow_flex",
  "wrist_flex",
  "wrist_roll",
];
const motorNames = [...jointNames, "gripper"];
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
const themeOptions = [
  {
    preference: "system",
    label: "System theme",
    icon: "M4 4h16v12H4zM8 20h8m-4-4v4",
  },
  {
    preference: "light",
    label: "Light theme",
    icon: "M12 2v2m0 16v2M2 12h2m16 0h2M5 5l1.5 1.5m11 11L19 19M5 19l1.5-1.5m11-11L19 5M16 12a4 4 0 1 1-8 0 4 4 0 0 1 8 0Z",
  },
  {
    preference: "dark",
    label: "Dark theme",
    icon: "M20.5 14A8.5 8.5 0 0 1 10 3.5 8.5 8.5 0 1 0 20.5 14Z",
  },
] as const;
const themeStorageKey = "so101-calibration-theme";

function readThemePreference(): ThemePreference {
  try {
    const stored = localStorage.getItem(themeStorageKey);
    return stored === "light" || stored === "dark" ? stored : "system";
  } catch {
    return "system";
  }
}

const programmingMotorIds = window.location.pathname === "/motor-ids";

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
  const [jointToRecalibrate, setJointToRecalibrate] = useState("");
  const acting = useRef(false);
  const eventStream = useRef<EventSource | null>(null);
  const [streamGeneration, setStreamGeneration] = useState(0);
  // biome-ignore lint/correctness/useExhaustiveDependencies: Reopen the stream after each action to discard queued pre-action events.
  useEffect(() => {
    if (programmingMotorIds) return;
    const stream = new EventSource("api/events");
    eventStream.current = stream;
    stream.addEventListener("view", (event: MessageEvent<string>) => {
      if (eventStream.current !== stream || acting.current) return;
      try {
        setView(viewSchema.parse(JSON.parse(event.data)));
        setConnectionError("");
      } catch (error) {
        setConnectionError(
          error instanceof Error ? error.message : String(error),
        );
      }
    });
    stream.addEventListener("view-error", (event: MessageEvent<string>) => {
      if (eventStream.current !== stream || acting.current) return;
      try {
        setConnectionError(
          z.object({ message: z.string() }).parse(JSON.parse(event.data))
            .message,
        );
      } catch {
        setConnectionError("Invalid response from the calibration server.");
      }
    });
    stream.onerror = () => {
      if (eventStream.current === stream && !acting.current)
        setConnectionError(
          "Connection lost. Reconnecting to the calibration server…",
        );
    };
    return () => {
      stream.close();
      if (eventStream.current === stream) eventStream.current = null;
    };
  }, [streamGeneration]);
  async function perform(action: Action) {
    if (acting.current) return;
    acting.current = true;
    eventStream.current?.close();
    eventStream.current = null;
    setBusy(true);
    setVerified(false);
    setActionError("");
    try {
      setView(await requestView(action));
      setJointToRecalibrate("");
      setConnectionError("");
    } catch (error) {
      setActionError(error instanceof Error ? error.message : String(error));
    } finally {
      acting.current = false;
      setBusy(false);
      setStreamGeneration((generation) => generation + 1);
    }
  }
  const stage = view?.stage ?? "disconnected";
  const connected = Boolean(view?.serial);
  const disabled = busy || Boolean(connectionError);
  const recalibratingJoint = view?.recalibrating_joint ?? null;
  const recalibratingName =
    recalibratingJoint === null
      ? null
      : labels[motorNames[recalibratingJoint] ?? ""];
  const showReference =
    stage === "reference" ||
    Boolean(view?.can_recalibrate_joint && jointToRecalibrate);
  const softGripper = view?.model_variant === "soft-gripper-camera";
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
        <a
          className="github-link"
          href="https://github.com/Hebbian-Robotics/so101"
          title="SO-101 GitHub repository"
        >
          <svg aria-hidden="true" viewBox="0 0 98 96" fill="currentColor">
            <path d="M41.4395 69.3848C28.8066 67.8535 19.9062 58.7617 19.9062 46.9902C19.9062 42.2051 21.6289 37.0371 24.5 33.5918C23.2559 30.4336 23.4473 23.7344 24.8828 20.959C28.7109 20.4805 33.8789 22.4902 36.9414 25.2656C40.5781 24.1172 44.4062 23.543 49.0957 23.543C53.7852 23.543 57.6133 24.1172 61.0586 25.1699C64.0254 22.4902 69.2891 20.4805 73.1172 20.959C74.457 23.543 74.6484 30.2422 73.4043 33.4961C76.4668 37.1328 78.0937 42.0137 78.0937 46.9902C78.0937 58.7617 69.1934 67.6621 56.3691 69.2891C59.623 71.3945 61.8242 75.9883 61.8242 81.252L61.8242 91.2051C61.8242 94.0762 64.2168 95.7031 67.0879 94.5547C84.4102 87.9512 98 70.6289 98 49.1914C98 22.1074 75.9883 6.69539e-07 48.9043 4.309e-07C21.8203 1.92261e-07 -1.9479e-07 22.1074 -4.3343e-07 49.1914C-6.20631e-07 70.4375 13.4941 88.0469 31.6777 94.6504C34.2617 95.6074 36.75 93.8848 36.75 91.3008L36.75 83.6445C35.4102 84.2188 33.6875 84.6016 32.1562 84.6016C25.8398 84.6016 22.1074 81.1563 19.4277 74.7441C18.375 72.1602 17.2266 70.6289 15.0254 70.3418C13.877 70.2461 13.4941 69.7676 13.4941 69.1934C13.4941 68.0449 15.4082 67.1836 17.3223 67.1836C20.0977 67.1836 22.4902 68.9063 24.9785 72.4473C26.8926 75.2227 28.9023 76.4668 31.2949 76.4668C33.6875 76.4668 35.2187 75.6055 37.4199 73.4043C39.0469 71.7773 40.291 70.3418 41.4395 69.3848Z" />
          </svg>
          <span className="visually-hidden">SO-101 GitHub repository</span>
        </a>
      </div>
      <div className="setup-layout">
        <nav className="setup-navigation" aria-label="SO-101 setup">
          <a href="/" aria-current={programmingMotorIds ? undefined : "page"}>
            Calibrate arm
          </a>
          <a
            href="/motor-ids"
            aria-current={programmingMotorIds ? "page" : undefined}
          >
            Program motor IDs
          </a>
        </nav>
        <div className="setup-content">
          <header className="page-heading">
            <div>
              <h1>
                {programmingMotorIds ? "Program motor IDs" : "Calibration"}
              </h1>
              <p>
                {programmingMotorIds
                  ? "Prepare loose motors for an SO-101 follower."
                  : "Calibrate and align this arm."}
              </p>
            </div>
            {!programmingMotorIds && (
              <span className="connection">
                {view?.serial || "Waiting for an arm"}
              </span>
            )}
          </header>
          {programmingMotorIds ? (
            <MotorIds />
          ) : (
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
                      Plug in the USB controller and power the arm with torque
                      off. A registered arm is selected automatically.
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
                {view &&
                (stage === "reference" ||
                  (stage === "sweep" && recalibratingJoint === null)) ? (
                  <section aria-labelledby="motor-order-heading">
                    <h2 id="motor-order-heading">Motor ID order</h2>
                    <p>
                      Current order:{" "}
                      {motorNames
                        .map(
                          (name, index) => view.motor_ids?.[name] ?? index + 1,
                        )
                        .join(" → ")}{" "}
                      from base to gripper.
                    </p>
                    <details>
                      <summary>Edit motor mapping</summary>
                      {stage === "sweep" ? (
                        <p>
                          Recorded travel follows the motor ID when you change
                          the mapping. Check the updated joint rows after
                          applying it.
                        </p>
                      ) : null}
                      <form
                        key={JSON.stringify(view.motor_ids)}
                        method="post"
                        action="api/action"
                        aria-labelledby="motor-mapping-legend"
                        onSubmit={(event) => {
                          event.preventDefault();
                          const form = new FormData(event.currentTarget);
                          void perform({
                            action: "map_motor_ids",
                            motor_ids: Object.fromEntries(
                              motorNames.map((name) => [
                                name,
                                Number(form.get(name)),
                              ]),
                            ),
                          });
                        }}
                      >
                        <fieldset disabled={disabled}>
                          <legend id="motor-mapping-legend">
                            Assign IDs 1–6 once each
                          </legend>
                          {motorNames.map((name, index) => (
                            <label
                              className="motor-field"
                              key={name}
                              htmlFor={`mapping-${name}`}
                            >
                              {labels[name]}
                              <select
                                id={`mapping-${name}`}
                                name={name}
                                defaultValue={
                                  view.motor_ids?.[name] ?? index + 1
                                }
                                required
                              >
                                {[1, 2, 3, 4, 5, 6].map((id) => (
                                  <option key={id} value={id}>
                                    {id}
                                  </option>
                                ))}
                              </select>
                            </label>
                          ))}
                          <button type="submit">Apply mapping</button>
                          {[false, true].map((descending) => (
                            <button
                              type="button"
                              key={String(descending)}
                              onClick={() =>
                                void perform({
                                  action: "map_motor_ids",
                                  motor_ids: Object.fromEntries(
                                    motorNames.map((name, index) => [
                                      name,
                                      descending ? 6 - index : index + 1,
                                    ]),
                                  ),
                                })
                              }
                            >
                              Use {descending ? "descending" : "ascending"}{" "}
                              order
                            </button>
                          ))}
                        </fieldset>
                      </form>
                    </details>
                    {stage === "reference" ? (
                      <>
                        {view.checking_motor_order ? (
                          <p role="status">
                            With torque off, gently turn only the base, then
                            check which motor moved.
                          </p>
                        ) : null}
                        {view.detected_base_motor !== null ? (
                          <>
                            <p role="status">
                              Base moved on motor {view.detected_base_motor}.
                            </p>
                            <button
                              type="button"
                              disabled={disabled}
                              onClick={() =>
                                void perform({
                                  action: "apply_detected_motor_order",
                                })
                              }
                            >
                              Use{" "}
                              {view.detected_base_motor === 1
                                ? "ascending"
                                : "descending"}{" "}
                              order
                            </button>
                          </>
                        ) : null}
                        <button
                          type="button"
                          disabled={disabled}
                          onClick={() =>
                            void perform({ action: "check_motor_order" })
                          }
                        >
                          {view.checking_motor_order
                            ? "Check which motor moved"
                            : "Check motor ID order"}
                        </button>
                      </>
                    ) : null}
                  </section>
                ) : null}
                {stage === "reference" && view?.offer_servo_reset ? (
                  <section aria-labelledby="servo-reset-heading">
                    <h2 id="servo-reset-heading">
                      Reset servos before calibration?
                    </h2>
                    <p>
                      Reset backs up and restores factory parameters on all six
                      servos. Motor IDs stay the same. The full sweep and save
                      will apply the new calibration and shared servo settings.
                    </p>
                    {view.servo_reset_incomplete ? (
                      <p role="alert">
                        The previous reset was incomplete. Retry before
                        calibrating.
                      </p>
                    ) : (
                      <button
                        type="button"
                        disabled={disabled}
                        onClick={() =>
                          void perform({ action: "skip_servo_reset" })
                        }
                      >
                        Skip reset
                      </button>
                    )}
                    <button
                      type="button"
                      disabled={disabled}
                      onClick={() => void perform({ action: "reset_servos" })}
                    >
                      {busy ? "Resetting servos…" : "Reset all six servos"}
                    </button>
                  </section>
                ) : null}
                {stage === "reference" && !view?.offer_servo_reset ? (
                  <>
                    <h2>Start calibration</h2>
                    {view?.message ? <p role="status">{view.message}</p> : null}
                    <p>
                      Support and disable the arm. Place it in the resting pose
                      shown, with the gripper fully closed. Start the sweep to
                      capture this reference, then gently move each joint
                      through its travel.
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
                {stage === "sweep" ? (
                  <>
                    <h2>
                      {recalibratingName
                        ? `Sweep ${recalibratingName}`
                        : "Record safe travel"}
                    </h2>
                    {recalibratingName ? (
                      <p>
                        Support the arm and slowly move the selected joint from
                        one gentle stop to the other.
                      </p>
                    ) : (
                      <p>
                        Support the arm. Slowly move each joint from one gentle
                        stop to the other, including wrist rotation. Fully close
                        and open the gripper without forcing it.
                      </p>
                    )}
                    <p>
                      Move slowly enough for the readings to follow. The model
                      appears after you finish the sweep.
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
                {stage === "review" ? (
                  <>
                    <h2>
                      {recalibratingName
                        ? `Review ${recalibratingName}`
                        : "Check the live model"}
                    </h2>
                    <p>
                      {recalibratingName
                        ? "Move the selected joint by hand. Check its direction and alignment in the model and adjust if needed."
                        : "Move the supported arm by hand. Check the joint directions, wrist orientation, and gripper. Adjust the model where needed."}
                    </p>
                    <label className="verification">
                      <input
                        type="checkbox"
                        checked={verified}
                        disabled={disabled || showReference}
                        onChange={(event) => setVerified(event.target.checked)}
                      />
                      The model follows this physical arm correctly.
                    </label>
                    <button
                      className="primary"
                      type="button"
                      disabled={disabled || showReference || !verified}
                      onClick={() =>
                        void perform({ action: "save", model_verified: true })
                      }
                    >
                      {busy
                        ? "Saving…"
                        : recalibratingName
                          ? "Save joint calibration"
                          : "Save setup"}
                    </button>
                    <p className="detail">
                      {recalibratingName
                        ? "Save backs up the existing setup and verifies the selected joint’s calibration with torque off."
                        : "Save backs up existing files, verifies calibration and gain registers with torque off, and updates this arm’s setup files."}
                    </p>
                    {view?.can_recalibrate_joint ? (
                      <form
                        aria-labelledby="recalibrate-joint-heading"
                        action="api/action"
                        method="post"
                        onSubmit={(event) => {
                          event.preventDefault();
                          if (!jointToRecalibrate) return;
                          void perform({
                            action: "start_sweep",
                            joint_index: Number(jointToRecalibrate),
                          });
                        }}
                      >
                        <h3 id="recalibrate-joint-heading">
                          Recalibrate one joint
                        </h3>
                        <label className="motor-field">
                          Joint to sweep
                          <select
                            name="joint_index"
                            required
                            disabled={disabled}
                            value={jointToRecalibrate}
                            onChange={(event) => {
                              setJointToRecalibrate(event.target.value);
                              setVerified(false);
                            }}
                          >
                            <option value="">Choose a joint</option>
                            {motorNames.map((name, index) => (
                              <option key={name} value={index}>
                                {labels[name]}
                              </option>
                            ))}
                          </select>
                        </label>
                        <p className="detail" id="joint-reference-instructions">
                          Support and disable the arm. Match the selected joint
                          to the reference pose before starting the sweep. For
                          the gripper, start fully closed.
                        </p>
                        <button
                          type="submit"
                          disabled={disabled}
                          aria-describedby="joint-reference-instructions"
                        >
                          Start joint sweep
                        </button>
                      </form>
                    ) : null}
                  </>
                ) : null}
                {stage === "saved" ? (
                  <>
                    <h2>Setup saved</h2>
                    <p>
                      The serial port is released. Calibration and model
                      alignment are saved in your selected setup directory.
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
                      If these files are in a shared configuration repository,
                      review and share the changes with your team.
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
                        {recalibratingName
                          ? "Discard joint calibration"
                          : "Start new calibration"}
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
                  This tool never enables torque or commands motion. Use your
                  control application for powered operation.
                </p>
              </section>
              <section
                className="visual-feedback"
                aria-label="Calibration feedback"
              >
                {stage === "sweep" ? (
                  <p role="status">The model appears after the sweep.</p>
                ) : (
                  <RobotScene
                    darkMode={darkMode}
                    modelUrl={
                      softGripper
                        ? "robot/so101/robot-soft-gripper-camera.urdf"
                        : "robot/so101/robot.urdf"
                    }
                    cameraPosition={calibrationCameraPosition}
                    cameraTarget={calibrationCameraTarget}
                    jointNames={jointNames}
                    measuredJoints={
                      showReference
                        ? (view?.home_joints_radians ?? [])
                        : (sceneArms[0]?.measuredJoints ?? [])
                    }
                    gripperAngleRadians={
                      showReference
                        ? view?.home_gripper_radians
                        : sceneArms[0]?.gripperAngleRadians
                    }
                  />
                )}
                {showReference ? <p>Resting pose reference</p> : null}
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
                          {stage !== "sweep" && <th>Angle</th>}
                          <th>Travel recorded</th>
                          {stage === "review" ? <th>Adjust model</th> : null}
                        </tr>
                      </thead>
                      <tbody>
                        {view.joints.map((joint, index) => (
                          <tr key={joint.name}>
                            <th scope="row">
                              {joint.moving ? (
                                <mark>
                                  {labels[joint.name] ?? joint.name} · Moving
                                </mark>
                              ) : (
                                (labels[joint.name] ?? joint.name)
                              )}
                              {stage === "sweep" &&
                              view.motor_ids?.[joint.name] !== undefined
                                ? ` · ID ${view.motor_ids[joint.name]}`
                                : null}
                            </th>
                            {stage !== "sweep" && (
                              <td>
                                {(
                                  (joint.position_radians * 180) /
                                  Math.PI
                                ).toFixed(1)}
                                °
                              </td>
                            )}
                            <td>
                              {joint.range_max > joint.range_min
                                ? `${(((joint.range_max - joint.range_min) * 360) / 4096).toFixed(0)}°`
                                : "Move this joint"}
                            </td>
                            {stage === "review" &&
                            (recalibratingJoint === null ||
                              recalibratingJoint === index) ? (
                              <td>
                                <button
                                  type="button"
                                  disabled={disabled}
                                  aria-label={`Reverse ${labels[joint.name] ?? joint.name}`}
                                  onClick={() =>
                                    void perform({
                                      action: "reverse_joint",
                                      index,
                                    })
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
                                          delta_radians:
                                            (direction * Math.PI) / 180,
                                        })
                                      }
                                    >
                                      {direction > 0 ? "+1°" : "−1°"}
                                    </button>
                                  ))}
                              </td>
                            ) : stage === "review" ? (
                              <td />
                            ) : null}
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  </div>
                ) : null}
              </section>
            </div>
          )}
        </div>
        <fieldset className="theme-control">
          <legend className="visually-hidden">Theme</legend>
          {themeOptions.map(({ preference, label, icon }) => (
            <button
              key={preference}
              className="theme-option"
              type="button"
              aria-label={label}
              aria-pressed={theme === preference}
              title={label}
              onClick={() => setTheme(preference)}
            >
              <svg
                aria-hidden="true"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="1.75"
                strokeLinecap="round"
                strokeLinejoin="round"
              >
                <path d={icon} />
              </svg>
            </button>
          ))}
        </fieldset>
      </div>
    </main>
  );
}

const root = document.getElementById("root");
if (!root) throw new Error("Calibration root element is missing");
createRoot(root).render(<App />);
