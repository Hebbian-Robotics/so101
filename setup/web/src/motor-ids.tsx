import { useCallback, useEffect, useState } from "react";
import { z } from "zod";

const programmingOptionsSchema = z.object({
  motors: z
    .array(z.object({ id: z.number().int(), joint: z.string() }))
    .length(6),
  ports: z.array(z.object({ port: z.string(), description: z.string() })),
});
type ProgrammingOptions = z.infer<typeof programmingOptionsSchema>;
type FollowerMotor = ProgrammingOptions["motors"][number];
type BatchResult = "verified" | "skipped";

const lastProgrammedMotorIdKey = "so101-last-programmed-motor-id";

function readLastProgrammedMotorId(): number | undefined {
  try {
    const stored = window.localStorage.getItem(lastProgrammedMotorIdKey);
    if (stored === null) return undefined;
    const motorId = Number(stored);
    return Number.isInteger(motorId) && motorId >= 1 && motorId <= 6
      ? motorId
      : undefined;
  } catch {
    return undefined;
  }
}

function rememberProgrammedMotorId(motorId: number): void {
  try {
    window.localStorage.setItem(lastProgrammedMotorIdKey, String(motorId));
  } catch {
    // The current selection remains available even if browser storage is disabled.
  }
}

function motorLabel(motor: FollowerMotor) {
  const joint = motor.joint.replaceAll("_", " ");
  return `${joint.charAt(0).toUpperCase()}${joint.slice(1)} (ID ${motor.id})`;
}

async function requestMotorIds(body?: object): Promise<unknown> {
  const response = await fetch(
    "/api/motor-ids",
    body && {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    },
  );
  const payload: unknown = await response.json();
  if (!response.ok) {
    const error = z.object({ message: z.string() }).safeParse(payload);
    throw new Error(
      error.success
        ? error.data.message
        : `Request failed (${response.status}).`,
    );
  }
  return payload;
}

function errorMessage(error: unknown) {
  return error instanceof Error ? error.message : String(error);
}

export function MotorIds() {
  const [options, setOptions] = useState<ProgrammingOptions>();
  const [selectedPort, setSelectedPort] = useState("");
  const [mode, setMode] = useState<"batch" | "single">("batch");
  const [batchResults, setBatchResults] = useState<BatchResult[]>([]);
  const [singleMotorId, setSingleMotorId] = useState(readLastProgrammedMotorId);
  const batchCount = batchResults.length;
  const [verifiedId, setVerifiedId] = useState<number>();
  const [isolated, setIsolated] = useState(false);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");

  const refreshAdapters = useCallback(async () => {
    setError("");
    try {
      setOptions(programmingOptionsSchema.parse(await requestMotorIds()));
    } catch (error) {
      setError(errorMessage(error));
    }
  }, []);

  useEffect(() => {
    void refreshAdapters();
  }, [refreshAdapters]);

  const port = options?.ports.some((adapter) => adapter.port === selectedPort)
    ? selectedPort
    : options?.ports.length === 1
      ? options.ports[0]?.port
      : "";
  const target =
    mode === "batch"
      ? options?.motors[batchCount]
      : options?.motors.find((motor) => motor.id === singleMotorId);

  function clearResult() {
    setIsolated(false);
    setVerifiedId(undefined);
    setError("");
  }

  async function programMotor() {
    if (busy || !port || !target || !isolated || verifiedId !== undefined)
      return;
    setBusy(true);
    setError("");
    try {
      const result = z.object({ motor_id: z.number().int() }).parse(
        await requestMotorIds({
          port,
          motor_id: target.id,
          isolated_motor: true,
        }),
      );
      if (result.motor_id !== target.id)
        throw new Error("Unexpected motor ID returned.");
      setVerifiedId(result.motor_id);
      if (mode === "single") {
        rememberProgrammedMotorId(result.motor_id);
        setSingleMotorId(result.motor_id);
      }
    } catch (error) {
      setError(errorMessage(error));
    } finally {
      setBusy(false);
    }
  }

  return (
    <div className="workspace motor-workspace">
      <section className="instructions" aria-label="Program a motor">
        <fieldset disabled={busy}>
          <legend>Job</legend>
          <label>
            <input
              type="radio"
              name="job"
              checked={mode === "batch"}
              onChange={() => {
                setMode("batch");
                clearResult();
              }}
            />
            New follower batch
          </label>
          <label>
            <input
              type="radio"
              name="job"
              checked={mode === "single"}
              onChange={() => {
                setMode("single");
                clearResult();
              }}
            />
            Program one ID
          </label>
        </fieldset>
        {mode === "batch" && options && batchCount === options.motors.length ? (
          <>
            <h2>Batch complete</h2>
            <p role="status">
              {batchResults.filter((result) => result === "verified").length}{" "}
              verified,{" "}
              {batchResults.filter((result) => result === "skipped").length}{" "}
              skipped.
            </p>
            <p>
              Only motors marked verified have been programmed in this batch.
            </p>
            <button
              type="button"
              onClick={() => {
                setBatchResults([]);
                clearResult();
              }}
            >
              Start another batch
            </button>
          </>
        ) : (
          <>
            {mode === "single" && (
              <label className="motor-field">
                Follower joint
                <select
                  disabled={busy}
                  value={singleMotorId ?? ""}
                  onChange={(event) => {
                    setSingleMotorId(Number(event.target.value));
                    clearResult();
                  }}
                >
                  <option value="" disabled>
                    Select a joint
                  </option>
                  {options?.motors.map((motor) => (
                    <option key={motor.id} value={motor.id}>
                      {motorLabel(motor)}
                    </option>
                  ))}
                </select>
              </label>
            )}
            {target && (
              <>
                <h2>{motorLabel(target)}</h2>
                {mode === "batch" && (
                  <p>
                    Motor {batchCount + 1} of {options?.motors.length}
                  </p>
                )}
              </>
            )}
            <label className="motor-field">
              USB servo adapter
              <select
                disabled={busy}
                value={port ?? ""}
                onChange={(event) => {
                  setSelectedPort(event.target.value);
                  clearResult();
                }}
              >
                <option value="" disabled>
                  {options?.ports.length
                    ? "Select an adapter"
                    : "No USB adapter detected"}
                </option>
                {options?.ports.map((adapter) => (
                  <option key={adapter.port} value={adapter.port}>
                    {adapter.description} ({adapter.port})
                  </option>
                ))}
              </select>
            </label>
            <button
              type="button"
              disabled={busy}
              onClick={() => void refreshAdapters()}
            >
              Refresh adapters
            </button>
            {target && verifiedId === undefined && (
              <>
                <p>
                  Turn off servo power before swapping motors. Connect one loose
                  STS3215 directly to the adapter, then turn servo power back
                  on.
                </p>
                <label className="motor-confirmation">
                  <input
                    type="checkbox"
                    checked={isolated}
                    disabled={busy}
                    onChange={(event) => setIsolated(event.target.checked)}
                  />
                  Only this loose motor is connected
                </label>
                <button
                  type="button"
                  disabled={busy || !port || !isolated}
                  onClick={() => void programMotor()}
                >
                  {busy ? "Programming…" : `Program ID ${target.id}`}
                </button>
              </>
            )}
            {mode === "batch" && target && verifiedId === undefined && (
              <button
                type="button"
                disabled={busy}
                onClick={() => {
                  setBatchResults((results) => [...results, "skipped"]);
                  clearResult();
                }}
              >
                Skip {motorLabel(target)}
              </button>
            )}
            {target && verifiedId === target.id && (
              <>
                <p role="status">
                  <strong>Verified: {motorLabel(target)}</strong>
                </p>
                <p>Label the motor, then turn off servo power and unplug it.</p>
                <button
                  type="button"
                  onClick={() => {
                    if (mode === "batch")
                      setBatchResults((results) => [...results, "verified"]);
                    clearResult();
                  }}
                >
                  {mode === "single"
                    ? `Program another ID ${target.id}`
                    : batchCount === 5
                      ? "Finish batch"
                      : "Next motor"}
                </button>
              </>
            )}
          </>
        )}
        {error && (
          <p role="alert" className="error">
            {error}
          </p>
        )}
      </section>
      <section className="visual-feedback">
        <h2>Follower motor IDs</h2>
        <ul className="motor-checklist">
          {options?.motors.map((motor, index) => (
            <li
              key={motor.id}
              aria-current={target?.id === motor.id ? "step" : undefined}
            >
              <span>{motorLabel(motor)}</span>
              {mode === "batch" && batchResults[index] === "skipped" ? (
                <strong>Skipped</strong>
              ) : (mode === "batch" && batchResults[index] === "verified") ||
                verifiedId === motor.id ? (
                <strong>Verified</strong>
              ) : null}
            </li>
          ))}
        </ul>
      </section>
    </div>
  );
}
