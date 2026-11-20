import { Cable, Cpu, SlidersHorizontal } from "lucide-react";
import { lazy, Suspense, useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import { match } from "ts-pattern";
import { z } from "zod";
import { Alert, AlertDescription } from "@/components/ui/alert";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Card, CardContent } from "@/components/ui/card";
import { Checkbox } from "@/components/ui/checkbox";
import {
  Field,
  FieldDescription,
  FieldGroup,
  FieldLabel,
  FieldLegend,
  FieldSet,
} from "@/components/ui/field";
import {
  NativeSelect,
  NativeSelectOption,
} from "@/components/ui/native-select";
import {
  Sidebar,
  SidebarContent,
  SidebarFooter,
  SidebarHeader,
  SidebarInset,
  SidebarMenu,
  SidebarMenuButton,
  SidebarMenuItem,
  SidebarProvider,
  SidebarTrigger,
  useSidebar,
} from "@/components/ui/sidebar";
import {
  Table,
  TableBody,
  TableCaption,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import { apiErrorSchema, requestJson } from "./api.js";
import hebbianLogo from "./hebbian-logo.svg";
import { MotorIds } from "./motor-ids.js";
import { ThemeMenu, type ThemePreference } from "./theme-menu.js";
import "./style.css";

const RobotScene = lazy(async () => {
  try {
    const { RobotScene } = await import("./robot-scene.js");
    return { default: RobotScene };
  } catch (error) {
    console.warn("Could not load the SO-101 visualization", error);
    return {
      default: () => (
        <p role="status">3D view unavailable. Joint values remain available.</p>
      ),
    };
  }
});

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
  estimated: z.boolean().default(false),
  can_recalibrate_joint: z.boolean(),
  can_reopen_saved_review: z.boolean().default(false),
  offer_servo_reset: z.boolean().default(false),
  motor_ids: z.record(z.string(), z.number().int().min(1).max(6)).optional(),
  checking_motor_order: z.boolean().default(false),
  detected_base_motor: z
    .union([z.literal(1), z.literal(6)])
    .nullable()
    .default(null),
  servo_reset_incomplete: z.boolean().default(false),
  recalibrating_joint: z.number().int().min(0).max(5).nullable(),
  review_revision: z.number().int().nonnegative().nullable(),
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
  | { action: "save"; model_verified: true; review_revision: number };
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
const restingCameraPosition: [number, number, number] = [0.43, -0.46, 0.32];
const restingCameraTarget: [number, number, number] = [0.03, 0, 0.08];

async function submitCalibrationAction(
  action: Action,
): Promise<CalibrationView> {
  return viewSchema.parse(await requestJson("/api/action", action));
}

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

function SetupNavigationTrigger() {
  const { isMobile, open, openMobile } = useSidebar();

  return (
    <SidebarTrigger
      aria-controls="so101-setup-navigation"
      aria-expanded={isMobile ? openMobile : open}
      aria-label="Toggle setup navigation"
      className="mb-3"
    />
  );
}
const initialTheme = readThemePreference();
if (initialTheme !== "system")
  document.documentElement.setAttribute("data-theme", initialTheme);

function App() {
  const [theme, setTheme] = useState<ThemePreference>(initialTheme);
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
  const [verifiedRevision, setVerifiedRevision] = useState<number | null>(null);
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
          apiErrorSchema.parse(JSON.parse(event.data)).message,
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
    setVerifiedRevision(null);
    setActionError("");
    try {
      setView(await submitCalibrationAction(action));
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
  const reviewRevision = view?.review_revision ?? null;
  const verified =
    verifiedRevision !== null && verifiedRevision === reviewRevision;
  const connected = Boolean(view?.serial);
  const previewingModel = stage === "disconnected";
  const disabled = busy || Boolean(connectionError);
  const recalibratingJoint = view?.recalibrating_joint ?? null;
  const recalibratingName =
    recalibratingJoint === null
      ? null
      : labels[motorNames[recalibratingJoint] ?? ""];
  const showReference =
    stage === "reference" ||
    Boolean(view?.can_recalibrate_joint && jointToRecalibrate);
  const showRestingPose = previewingModel || showReference;
  const softGripper = view?.model_variant === "soft-gripper-camera";
  const measuredAngles = new Map(
    view?.joints.map((joint) => [joint.name, joint.position_radians]),
  );
  return (
    <SidebarProvider>
      <Sidebar>
        <SidebarHeader className="p-3 max-md:pr-14">
          <div className="flex min-w-0 items-center gap-2 text-sm">
            <img src={hebbianLogo} alt="" className="size-8 shrink-0" />
            <span>
              Hebbian <strong>Robot Use</strong>
            </span>
          </div>
        </SidebarHeader>
        <SidebarContent className="px-2">
          <nav id="so101-setup-navigation" aria-label="SO-101 setup">
            <SidebarMenu>
              <SidebarMenuItem>
                <SidebarMenuButton
                  render={<a href="/" />}
                  aria-current={programmingMotorIds ? undefined : "page"}
                  isActive={!programmingMotorIds}
                >
                  <SlidersHorizontal aria-hidden="true" />
                  Calibrate arm
                </SidebarMenuButton>
              </SidebarMenuItem>
              <SidebarMenuItem>
                <SidebarMenuButton
                  render={<a href="/motor-ids" />}
                  aria-current={programmingMotorIds ? "page" : undefined}
                  isActive={programmingMotorIds}
                >
                  <Cpu aria-hidden="true" />
                  Program motor IDs
                </SidebarMenuButton>
              </SidebarMenuItem>
            </SidebarMenu>
          </nav>
        </SidebarContent>
        <SidebarFooter className="flex-row items-center">
          <SidebarMenuButton
            className="size-11 justify-center text-muted-foreground"
            render={<a href="https://github.com/Hebbian-Robotics/so101" />}
            aria-label="SO-101 GitHub repository"
          >
            <svg aria-hidden="true" viewBox="0 0 98 96" fill="currentColor">
              <path d="M41.4395 69.3848C28.8066 67.8535 19.9062 58.7617 19.9062 46.9902C19.9062 42.2051 21.6289 37.0371 24.5 33.5918C23.2559 30.4336 23.4473 23.7344 24.8828 20.959C28.7109 20.4805 33.8789 22.4902 36.9414 25.2656C40.5781 24.1172 44.4062 23.543 49.0957 23.543C53.7852 23.543 57.6133 24.1172 61.0586 25.1699C64.0254 22.4902 69.2891 20.4805 73.1172 20.959C74.457 23.543 74.6484 30.2422 73.4043 33.4961C76.4668 37.1328 78.0937 42.0137 78.0937 46.9902C78.0937 58.7617 69.1934 67.6621 56.3691 69.2891C59.623 71.3945 61.8242 75.9883 61.8242 81.252L61.8242 91.2051C61.8242 94.0762 64.2168 95.7031 67.0879 94.5547C84.4102 87.9512 98 70.6289 98 49.1914C98 22.1074 75.9883 6.69539e-07 48.9043 4.309e-07C21.8203 1.92261e-07 -1.9479e-07 22.1074 -4.3343e-07 49.1914C-6.20631e-07 70.4375 13.4941 88.0469 31.6777 94.6504C34.2617 95.6074 36.75 93.8848 36.75 91.3008L36.75 83.6445C35.4102 84.2188 33.6875 84.6016 32.1562 84.6016C25.8398 84.6016 22.1074 81.1563 19.4277 74.7441C18.375 72.1602 17.2266 70.6289 15.0254 70.3418C13.877 70.2461 13.4941 69.7676 13.4941 69.1934C13.4941 68.0449 15.4082 67.1836 17.3223 67.1836C20.0977 67.1836 22.4902 68.9063 24.9785 72.4473C26.8926 75.2227 28.9023 76.4668 31.2949 76.4668C33.6875 76.4668 35.2187 75.6055 37.4199 73.4043C39.0469 71.7773 40.291 70.3418 41.4395 69.3848Z" />
            </svg>
          </SidebarMenuButton>
          <ThemeMenu theme={theme} onThemeChange={setTheme} />
        </SidebarFooter>
      </Sidebar>

      <SidebarInset className="setup-content min-w-0">
        <SetupNavigationTrigger />
        <header className="page-heading">
          <div>
            <h1>{programmingMotorIds ? "Program motor IDs" : "Calibration"}</h1>
            <p>
              {programmingMotorIds
                ? "Prepare loose motors for an SO-101 follower."
                : "Record joint travel and verify model alignment."}
            </p>
          </div>
          {!programmingMotorIds && (
            <Badge
              variant={connected ? "default" : "outline"}
              className="h-auto max-w-full whitespace-normal wrap-anywhere"
            >
              <Cable aria-hidden="true" />
              {view?.serial || "Waiting for an arm"}
            </Badge>
          )}
        </header>
        {programmingMotorIds ? (
          <MotorIds />
        ) : (
          <div className="workspace">
            <Card
              size="sm"
              className="min-w-0"
              role="region"
              aria-label="Calibration steps"
            >
              <CardContent className="block">
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
                  <Alert variant="destructive" className="my-3">
                    <AlertDescription>{connectionError}</AlertDescription>
                  </Alert>
                ) : null}
                {actionError ? (
                  <Alert variant="destructive" className="my-3">
                    <AlertDescription>{actionError}</AlertDescription>
                  </Alert>
                ) : null}
                {match({
                  stage,
                  offerServoReset: view?.offer_servo_reset === true,
                })
                  .with({ stage: "disconnected" }, () => (
                    <>
                      <h2>Connect your arm</h2>
                      <p>
                        Plug in the USB controller and power the arm with torque
                        off.
                      </p>
                      {view?.controllers.map((controller) => (
                        <Button
                          variant="outline"
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
                        </Button>
                      ))}
                      <p className="detail">
                        {view?.message || "Looking for USB controllers…"}
                      </p>
                    </>
                  ))
                  .with({ stage: "reference", offerServoReset: true }, () => (
                    <section aria-labelledby="servo-reset-heading">
                      <h2 id="servo-reset-heading">
                        Reset servos before full calibration?
                      </h2>
                      {view?.message && !view.servo_reset_incomplete ? (
                        <p role="status">{view.message}</p>
                      ) : null}
                      <p>
                        Reset backs up and restores factory parameters on all
                        six servos. Motor IDs stay the same. The full sweep and
                        save will apply the new calibration and shared servo
                        settings.
                      </p>
                      {view?.servo_reset_incomplete ? (
                        <Alert variant="destructive" className="my-3">
                          <AlertDescription>
                            The previous reset was incomplete. Retry before
                            calibrating.
                          </AlertDescription>
                        </Alert>
                      ) : null}
                      <div className="flex flex-wrap items-center gap-2">
                        <Button
                          variant="outline"
                          type="button"
                          disabled={disabled}
                          onClick={() =>
                            void perform({ action: "reset_servos" })
                          }
                        >
                          {busy ? "Resetting servos…" : "Reset all six servos"}
                        </Button>
                        {view?.servo_reset_incomplete ? null : (
                          <Button
                            variant="link"
                            size="sm"
                            className="text-xs underline hover:decoration-2"
                            type="button"
                            disabled={disabled}
                            onClick={() =>
                              void perform({ action: "skip_servo_reset" })
                            }
                          >
                            Skip reset
                          </Button>
                        )}
                      </div>
                    </section>
                  ))
                  .with({ stage: "reference", offerServoReset: false }, () => (
                    <>
                      <h2>Start full calibration</h2>
                      {view?.message ? (
                        <p role="status">{view.message}</p>
                      ) : null}
                      <p>
                        Support and disable the arm. Place it in the resting
                        pose shown, with the gripper fully closed. Start the
                        sweep to capture this reference, then gently move each
                        joint through its travel.
                      </p>
                      <Button
                        className="mt-2.5 w-full"
                        size="lg"
                        type="button"
                        disabled={disabled}
                        onClick={() => void perform({ action: "start_sweep" })}
                      >
                        Start sweep
                      </Button>
                    </>
                  ))
                  .with({ stage: "sweep" }, () => (
                    <>
                      {view && recalibratingJoint === null ? (
                        <section aria-labelledby="motor-order-heading">
                          <h2 id="motor-order-heading">Motor ID order</h2>
                          <p>
                            Current order:{" "}
                            {motorNames
                              .map(
                                (name, index) =>
                                  view.motor_ids?.[name] ?? index + 1,
                              )
                              .join(" → ")}{" "}
                            from base to gripper.
                          </p>
                          <details>
                            <summary>Edit motor mapping</summary>
                            <p>
                              Recorded travel follows the motor ID when you
                              change the mapping. Check the updated joint rows
                              after applying it.
                            </p>
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
                              <FieldSet disabled={disabled} className="my-3">
                                <FieldLegend id="motor-mapping-legend">
                                  Assign IDs 1–6 once each
                                </FieldLegend>
                                <FieldGroup className="gap-3">
                                  {motorNames.map((name, index) => (
                                    <Field key={name}>
                                      <FieldLabel htmlFor={`mapping-${name}`}>
                                        {labels[name]}
                                      </FieldLabel>
                                      <NativeSelect
                                        id={`mapping-${name}`}
                                        name={name}
                                        defaultValue={
                                          view.motor_ids?.[name] ?? index + 1
                                        }
                                        required
                                      >
                                        {[1, 2, 3, 4, 5, 6].map((id) => (
                                          <NativeSelectOption
                                            key={id}
                                            value={id}
                                          >
                                            {id}
                                          </NativeSelectOption>
                                        ))}
                                      </NativeSelect>
                                    </Field>
                                  ))}
                                </FieldGroup>
                                <div className="flex flex-wrap gap-2">
                                  <Button variant="outline" type="submit">
                                    Apply mapping
                                  </Button>
                                  {[false, true].map((descending) => (
                                    <Button
                                      variant="outline"
                                      type="button"
                                      key={String(descending)}
                                      onClick={() =>
                                        void perform({
                                          action: "map_motor_ids",
                                          motor_ids: Object.fromEntries(
                                            motorNames.map((name, index) => [
                                              name,
                                              descending
                                                ? 6 - index
                                                : index + 1,
                                            ]),
                                          ),
                                        })
                                      }
                                    >
                                      Use{" "}
                                      {descending ? "descending" : "ascending"}{" "}
                                      order
                                    </Button>
                                  ))}
                                </div>
                              </FieldSet>
                            </form>
                          </details>
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
                              <Button
                                variant="outline"
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
                              </Button>
                            </>
                          ) : null}
                          <Button
                            variant="outline"
                            type="button"
                            disabled={disabled}
                            onClick={() =>
                              void perform({ action: "check_motor_order" })
                            }
                          >
                            {view.checking_motor_order
                              ? "Check which motor moved"
                              : "Check motor ID order"}
                          </Button>
                        </section>
                      ) : null}
                      <h2>
                        {recalibratingName
                          ? `Sweep ${recalibratingName}`
                          : "Record safe travel"}
                      </h2>
                      {recalibratingName ? (
                        <p>
                          Support the arm and slowly move the selected joint
                          from one gentle stop to the other.
                        </p>
                      ) : (
                        <p>
                          Support the arm. Slowly move each joint from one
                          gentle stop to the other, including wrist rotation.
                          Fully close and open the gripper without forcing it.
                        </p>
                      )}
                      <p>
                        Move slowly enough for the readings to follow. The model
                        appears after you finish the sweep.
                      </p>
                      <Button
                        className="mt-2.5 w-full"
                        size="lg"
                        type="button"
                        disabled={disabled}
                        onClick={() => void perform({ action: "finish_sweep" })}
                      >
                        Finish recording
                      </Button>
                    </>
                  ))
                  .with({ stage: "review" }, () => (
                    <>
                      {view?.can_recalibrate_joint ? (
                        <form
                          className="mb-5 border-b pb-5"
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
                          <h2 id="recalibrate-joint-heading">
                            Recalibrate one joint
                          </h2>
                          <Field className="my-3">
                            <FieldLabel htmlFor="calibration-joint-index">
                              Joint to sweep
                            </FieldLabel>
                            <NativeSelect
                              id="calibration-joint-index"
                              aria-describedby="joint-reference-instructions"
                              name="joint_index"
                              required
                              disabled={disabled}
                              value={jointToRecalibrate}
                              onChange={(event) => {
                                setJointToRecalibrate(event.target.value);
                                setVerifiedRevision(null);
                              }}
                            >
                              <NativeSelectOption value="">
                                Choose a joint
                              </NativeSelectOption>
                              {motorNames.map((name, index) => (
                                <NativeSelectOption key={name} value={index}>
                                  {labels[name]}
                                </NativeSelectOption>
                              ))}
                            </NativeSelect>
                            <FieldDescription id="joint-reference-instructions">
                              Support and disable the arm. Match the selected
                              joint to the reference pose before starting the
                              sweep. For the gripper, start fully closed.
                            </FieldDescription>
                          </Field>
                          <Button
                            type="submit"
                            disabled={disabled}
                            aria-describedby="joint-reference-instructions"
                          >
                            Start joint sweep
                          </Button>
                        </form>
                      ) : null}
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
                      <Field orientation="horizontal" className="my-3">
                        <Checkbox
                          id="model-verified"
                          checked={verified}
                          disabled={disabled || showReference}
                          onCheckedChange={(checked) =>
                            setVerifiedRevision(checked ? reviewRevision : null)
                          }
                        />
                        <FieldLabel htmlFor="model-verified">
                          The model follows this physical arm correctly.
                        </FieldLabel>
                      </Field>
                      <Button
                        className="mt-2.5 w-full"
                        size="lg"
                        type="button"
                        disabled={disabled || showReference || !verified}
                        onClick={() => {
                          if (verifiedRevision === null || !verified) return;
                          void perform({
                            action: "save",
                            model_verified: true,
                            review_revision: verifiedRevision,
                          });
                        }}
                      >
                        {busy
                          ? "Saving…"
                          : recalibratingName
                            ? "Save joint calibration"
                            : "Save setup"}
                      </Button>
                      <p className="detail">
                        {recalibratingName
                          ? "Save backs up the existing setup and verifies the selected joint’s calibration with torque off."
                          : "Save backs up existing files, verifies calibration and gain registers with torque off, and updates this arm’s setup files."}
                      </p>
                    </>
                  ))
                  .with({ stage: "saved" }, () => (
                    <>
                      <h2>Setup saved</h2>
                      <p>
                        The serial port is released. Calibration and model
                        alignment are saved in your selected setup directory.
                      </p>
                      <Button
                        variant="outline"
                        type="button"
                        disabled={disabled}
                        onClick={() => void perform({ action: "reopen" })}
                      >
                        Review or recalibrate saved setup
                      </Button>
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
                  ))
                  .with({ stage: "error" }, () => (
                    <>
                      <h2>Inspect the arm before continuing</h2>
                      <Alert variant="destructive" className="my-3">
                        <AlertDescription>{view?.message}</AlertDescription>
                      </Alert>
                    </>
                  ))
                  .exhaustive()}
                {connected && stage !== "saved" && stage !== "error" ? (
                  <div className="secondary-actions">
                    {stage === "reference" && view?.can_reopen_saved_review ? (
                      <Button
                        variant="link"
                        className="underline hover:decoration-2"
                        type="button"
                        disabled={disabled}
                        onClick={() => void perform({ action: "reopen" })}
                      >
                        Back to saved review
                      </Button>
                    ) : null}
                    {stage !== "reference" ? (
                      <Button
                        variant="outline"
                        type="button"
                        disabled={disabled}
                        onClick={() => {
                          void perform({ action: "restart" });
                        }}
                      >
                        {recalibratingName
                          ? "Discard joint calibration"
                          : "Start full calibration"}
                      </Button>
                    ) : null}
                    <Button
                      variant="outline"
                      type="button"
                      disabled={busy}
                      onClick={() => void perform({ action: "disconnect" })}
                    >
                      Discard and disconnect
                    </Button>
                  </div>
                ) : null}
                <p className="detail mt-4 mb-0 border-t pt-3">
                  This tool never enables torque or commands motion. Use your
                  control application for powered operation.
                </p>
              </CardContent>
            </Card>
            <Card
              size="sm"
              role="region"
              className="min-w-0"
              aria-label="Calibration feedback"
            >
              <CardContent className="block">
                {stage === "sweep" ? (
                  <p role="status">The model appears after the sweep.</p>
                ) : !view ? (
                  <p role="status">Waiting for the calibration server…</p>
                ) : (
                  <Suspense
                    fallback={<p role="status">Loading SO-101 model…</p>}
                  >
                    <RobotScene
                      darkMode={darkMode}
                      title={
                        previewingModel
                          ? "Resting pose preview"
                          : showReference
                            ? "Resting pose reference"
                            : stage === "saved" || stage === "error"
                              ? "Last recorded pose"
                              : "Live robot model"
                      }
                      modelUrl={
                        softGripper
                          ? "robot/so101/robot-soft-gripper-camera.urdf"
                          : "robot/so101/robot.urdf"
                      }
                      cameraPosition={
                        showRestingPose
                          ? restingCameraPosition
                          : calibrationCameraPosition
                      }
                      cameraTarget={
                        showRestingPose
                          ? restingCameraTarget
                          : calibrationCameraTarget
                      }
                      jointNames={jointNames}
                      measuredJoints={
                        showRestingPose
                          ? view.home_joints_radians
                          : jointNames.map(
                              (name) => measuredAngles.get(name) ?? null,
                            )
                      }
                      gripperAngleRadians={
                        showRestingPose
                          ? view.home_gripper_radians
                          : measuredAngles.get("gripper")
                      }
                    />
                  </Suspense>
                )}
                {stage !== "reference" && view?.joints.length ? (
                  <Table
                    className="mt-4 tabular-nums"
                    scrollLabelledBy="joint-readings-caption"
                  >
                    <TableCaption
                      id="joint-readings-caption"
                      className="caption-top text-left"
                    >
                      {stage === "saved"
                        ? "Last readings before saving"
                        : "Live joint readings and recorded encoder travel"}
                    </TableCaption>
                    <TableHeader>
                      <TableRow>
                        <TableHead scope="col">Joint</TableHead>
                        {stage !== "sweep" && (
                          <TableHead scope="col">Angle</TableHead>
                        )}
                        <TableHead scope="col">Travel recorded</TableHead>
                        {stage === "review" ? (
                          <TableHead scope="col">Adjust model</TableHead>
                        ) : null}
                      </TableRow>
                    </TableHeader>
                    <TableBody>
                      {view.joints.map((joint, index) => (
                        <TableRow
                          key={joint.name}
                          className={joint.moving ? "joint-active" : undefined}
                        >
                          <TableHead scope="row">
                            {labels[joint.name] ?? joint.name}
                            {stage === "sweep" &&
                            view.motor_ids?.[joint.name] !== undefined
                              ? ` · ID ${view.motor_ids[joint.name]}`
                              : null}
                          </TableHead>
                          {stage !== "sweep" && (
                            <TableCell>
                              {(
                                (joint.position_radians * 180) /
                                Math.PI
                              ).toFixed(1)}
                              °
                            </TableCell>
                          )}
                          <TableCell>
                            {joint.range_max > joint.range_min
                              ? `${(((joint.range_max - joint.range_min) * 360) / 4096).toFixed(0)}°`
                              : "Move this joint"}
                          </TableCell>
                          {stage === "review" &&
                          (recalibratingJoint === null ||
                            recalibratingJoint === index) ? (
                            <TableCell>
                              <div className="flex flex-wrap gap-1">
                                <Button
                                  variant="outline"
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
                                </Button>
                                {stage === "review" &&
                                  [-1, 1].map((direction) => (
                                    <Button
                                      variant="outline"
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
                                    </Button>
                                  ))}
                              </div>
                            </TableCell>
                          ) : stage === "review" ? (
                            <TableCell />
                          ) : null}
                        </TableRow>
                      ))}
                    </TableBody>
                  </Table>
                ) : null}
              </CardContent>
            </Card>
          </div>
        )}
      </SidebarInset>
    </SidebarProvider>
  );
}

const root = document.getElementById("root");
if (!root) throw new Error("Calibration root element is missing");
createRoot(root).render(<App />);
