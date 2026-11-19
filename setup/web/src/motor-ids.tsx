import { useMachine } from "@xstate/react";
import { RefreshCw } from "lucide-react";
import { Button } from "@/components/ui/button";
import { Checkbox } from "@/components/ui/checkbox";
import {
  NativeSelect,
  NativeSelectOption,
} from "@/components/ui/native-select";
import {
  type FollowerMotor,
  getWizardSelection,
  motorIdWizard,
} from "./motor-id-wizard.js";

function motorLabel(motor: FollowerMotor) {
  const joint = motor.joint.replaceAll("_", " ");
  return `${joint.charAt(0).toUpperCase()}${joint.slice(1)} (ID ${motor.id})`;
}

export function MotorIds() {
  const [snapshot, send] = useMachine(motorIdWizard);
  const {
    options,
    mode,
    batchResults,
    singleMotorId,
    verifiedMotorId: verifiedId,
    error,
  } = snapshot.context;
  const batchCount = batchResults.length;
  const { port, motor: target } = getWizardSelection(snapshot.context);
  const refreshingAdapters = snapshot.matches("refreshing");
  const programming = snapshot.matches("programming");
  const busy = refreshingAdapters || programming;
  const isolated = snapshot.matches({ interactive: "confirmed" });

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
              onChange={() => send({ type: "selectMode", mode: "batch" })}
            />
            New follower batch
          </label>
          <label>
            <input
              type="radio"
              name="job"
              checked={mode === "single"}
              onChange={() => send({ type: "selectMode", mode: "single" })}
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
            <Button
              variant="outline"
              type="button"
              onClick={() => send({ type: "startNewBatch" })}
            >
              Start another batch
            </Button>
          </>
        ) : (
          <>
            {mode === "single" && (
              <div className="motor-field">
                <label htmlFor="follower-joint-id">Follower joint</label>
                <NativeSelect
                  id="follower-joint-id"
                  disabled={busy}
                  value={singleMotorId ?? ""}
                  onChange={(event) =>
                    send({
                      type: "selectMotor",
                      motorId: Number(event.target.value),
                    })
                  }
                >
                  <NativeSelectOption value="" disabled>
                    Select a joint
                  </NativeSelectOption>
                  {options?.motors.map((motor) => (
                    <NativeSelectOption key={motor.id} value={motor.id}>
                      {motorLabel(motor)}
                    </NativeSelectOption>
                  ))}
                </NativeSelect>
              </div>
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
            <div className="motor-field">
              <label htmlFor="usb-servo-adapter">USB servo adapter</label>
              <NativeSelect
                id="usb-servo-adapter"
                disabled={busy}
                value={port ?? ""}
                onChange={(event) =>
                  send({ type: "selectPort", port: event.target.value })
                }
              >
                <NativeSelectOption value="" disabled>
                  {options?.ports.length
                    ? "Select an adapter"
                    : "No USB adapter detected"}
                </NativeSelectOption>
                {options?.ports.map((adapter) => (
                  <NativeSelectOption key={adapter.port} value={adapter.port}>
                    {adapter.description} ({adapter.port})
                  </NativeSelectOption>
                ))}
              </NativeSelect>
            </div>
            <Button
              variant="outline"
              type="button"
              disabled={busy}
              onClick={() => send({ type: "refreshAdapters" })}
            >
              <RefreshCw aria-hidden="true" /> Refresh adapters
            </Button>
            {refreshingAdapters && <p role="status">Checking USB adapters…</p>}
            {target && verifiedId === undefined && (
              <>
                <p>
                  Turn off servo power before swapping motors. Connect one loose
                  STS3215 directly to the adapter, then turn servo power back
                  on.
                </p>
                <label className="motor-confirmation" htmlFor="motor-isolated">
                  <Checkbox
                    id="motor-isolated"
                    checked={isolated}
                    disabled={busy || !port}
                    onCheckedChange={(checked) =>
                      send({
                        type:
                          checked === true
                            ? "confirmIsolation"
                            : "withdrawConfirmation",
                      })
                    }
                  />
                  Only this loose motor is connected
                </label>
                <Button
                  className="mt-2.5 w-full"
                  size="lg"
                  type="button"
                  disabled={busy || !port || !isolated}
                  onClick={() => send({ type: "programMotor" })}
                >
                  {programming ? "Programming…" : `Program ID ${target.id}`}
                </Button>
              </>
            )}
            {mode === "batch" && target && verifiedId === undefined && (
              <Button
                variant="outline"
                type="button"
                disabled={busy}
                onClick={() => send({ type: "skipMotor" })}
              >
                Skip {motorLabel(target)}
              </Button>
            )}
            {target && verifiedId === target.id && (
              <>
                <p role="status">
                  <strong>Verified: {motorLabel(target)}</strong>
                </p>
                <p>Label the motor, then turn off servo power and unplug it.</p>
                <Button
                  variant="outline"
                  type="button"
                  disabled={busy}
                  onClick={() => send({ type: "continueAfterVerification" })}
                >
                  {mode === "single"
                    ? `Program another ID ${target.id}`
                    : batchCount === 5
                      ? "Finish batch"
                      : "Next motor"}
                </Button>
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
                <strong className="motor-result">Skipped</strong>
              ) : (mode === "batch" && batchResults[index] === "verified") ||
                verifiedId === motor.id ? (
                <strong className="motor-result" data-verified>
                  Verified
                </strong>
              ) : null}
            </li>
          ))}
        </ul>
      </section>
    </div>
  );
}
