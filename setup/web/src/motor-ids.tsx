import { useMachine } from "@xstate/react";
import { RefreshCw } from "lucide-react";
import { Alert, AlertDescription } from "@/components/ui/alert";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Checkbox } from "@/components/ui/checkbox";
import {
  Field,
  FieldLabel,
  FieldLegend,
  FieldSet,
} from "@/components/ui/field";
import {
  NativeSelect,
  NativeSelectOption,
} from "@/components/ui/native-select";
import { RadioGroup, RadioGroupItem } from "@/components/ui/radio-group";
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
      <Card
        size="sm"
        className="min-w-0"
        role="region"
        aria-label="Program a motor"
      >
        <CardContent className="block">
          <FieldSet>
            <FieldLegend id="motor-programming-job">Job</FieldLegend>
            <RadioGroup
              aria-labelledby="motor-programming-job"
              disabled={busy}
              name="job"
              value={mode}
              onValueChange={(mode) => send({ type: "selectMode", mode })}
            >
              <Field orientation="horizontal">
                <RadioGroupItem id="motor-programming-batch" value="batch" />
                <FieldLabel htmlFor="motor-programming-batch">
                  New follower batch
                </FieldLabel>
              </Field>
              <Field orientation="horizontal">
                <RadioGroupItem id="motor-programming-single" value="single" />
                <FieldLabel htmlFor="motor-programming-single">
                  Program one ID
                </FieldLabel>
              </Field>
            </RadioGroup>
          </FieldSet>
          {mode === "batch" &&
          options &&
          batchCount === options.motors.length ? (
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
                <Field className="my-3">
                  <FieldLabel htmlFor="follower-joint-id">
                    Follower joint
                  </FieldLabel>
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
                </Field>
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
              <Field className="my-3">
                <FieldLabel htmlFor="usb-servo-adapter">
                  USB servo adapter
                </FieldLabel>
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
              </Field>
              <Button
                variant="outline"
                type="button"
                disabled={busy}
                onClick={() => send({ type: "refreshAdapters" })}
              >
                <RefreshCw aria-hidden="true" /> Refresh adapters
              </Button>
              {refreshingAdapters && (
                <p role="status">Checking USB adapters…</p>
              )}
              {target && verifiedId === undefined && (
                <>
                  <p>
                    Turn off servo power before swapping motors. Connect one
                    loose STS3215 directly to the adapter, then turn servo power
                    back on.
                  </p>
                  <Field orientation="horizontal" className="my-3">
                    <Checkbox
                      id="motor-isolated"
                      checked={isolated}
                      disabled={busy || !port}
                      onCheckedChange={(checked) =>
                        send({
                          type: checked
                            ? "confirmIsolation"
                            : "withdrawConfirmation",
                        })
                      }
                    />
                    <FieldLabel htmlFor="motor-isolated">
                      Only this loose motor is connected
                    </FieldLabel>
                  </Field>
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
                  <p>
                    Label the motor, then turn off servo power and unplug it.
                  </p>
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
            <Alert variant="destructive" className="my-3">
              <AlertDescription>{error}</AlertDescription>
            </Alert>
          )}
        </CardContent>
      </Card>
      <Card
        size="sm"
        className="min-w-0"
        role="region"
        aria-labelledby="follower-motors-heading"
      >
        <CardHeader>
          <CardTitle id="follower-motors-heading" role="heading" aria-level={2}>
            Follower motor IDs
          </CardTitle>
        </CardHeader>
        <CardContent>
          <ul className="divide-y">
            {options?.motors.map((motor, index) => (
              <li
                key={motor.id}
                aria-current={target?.id === motor.id ? "step" : undefined}
                className="flex min-h-11 items-center justify-between gap-2 rounded-md px-2 py-1.5 aria-[current=step]:bg-accent aria-[current=step]:font-medium aria-[current=step]:text-accent-foreground"
              >
                <span>{motorLabel(motor)}</span>
                {mode === "batch" && batchResults[index] === "skipped" ? (
                  <Badge variant="secondary">Skipped</Badge>
                ) : (mode === "batch" && batchResults[index] === "verified") ||
                  verifiedId === motor.id ? (
                  <Badge>Verified</Badge>
                ) : null}
              </li>
            ))}
          </ul>
        </CardContent>
      </Card>
    </div>
  );
}
