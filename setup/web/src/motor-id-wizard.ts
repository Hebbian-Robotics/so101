import { assign, fromPromise, setup } from "xstate";
import { z } from "zod";
import { requestJson } from "./api.js";

const motorIdsPath = "/api/motor-ids";
const motorIdSchema = z.number().int().min(1).max(6);
const programmingOptionsSchema = z.object({
  motors: z.array(z.object({ id: motorIdSchema, joint: z.string() })).length(6),
  ports: z.array(z.object({ port: z.string(), description: z.string() })),
});
const programmedMotorSchema = z.object({ motor_id: motorIdSchema });

type ProgrammingOptions = z.infer<typeof programmingOptionsSchema>;
export type FollowerMotor = ProgrammingOptions["motors"][number];
type BatchResult = "verified" | "skipped";
type ProgrammingMode = "batch" | "single";
type ProgrammingTarget = { port: string; motorId: number };

type WizardContext = {
  options?: ProgrammingOptions;
  selectedPort: string;
  mode: ProgrammingMode;
  batchResults: BatchResult[];
  singleMotorId?: number;
  verifiedMotorId?: number;
  error: string;
};

type WizardEvent =
  | { type: "selectMode"; mode: ProgrammingMode }
  | { type: "selectPort"; port: string }
  | { type: "selectMotor"; motorId: number }
  | { type: "refreshAdapters" }
  | { type: "confirmIsolation" }
  | { type: "withdrawConfirmation" }
  | { type: "programMotor" }
  | { type: "skipMotor" }
  | { type: "continueAfterVerification" }
  | { type: "startNewBatch" };

const lastProgrammedMotorIdKey = "so101-last-programmed-motor-id";

function readLastProgrammedMotorId(): number | undefined {
  try {
    const stored = window.localStorage.getItem(lastProgrammedMotorIdKey);
    if (stored === null) return undefined;
    return motorIdSchema.safeParse(Number(stored)).data;
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

export function getWizardSelection(context: WizardContext) {
  const { options, selectedPort, batchResults, mode, singleMotorId } = context;
  const port = options?.ports.some((adapter) => adapter.port === selectedPort)
    ? selectedPort
    : options?.ports.length === 1
      ? options.ports[0]?.port
      : undefined;
  const motor =
    mode === "batch"
      ? options?.motors[batchResults.length]
      : options?.motors.find((candidate) => candidate.id === singleMotorId);
  return { port, motor };
}

function errorMessage(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

const wizard = setup({
  types: {
    context: {} as WizardContext,
    events: {} as WizardEvent,
  },
  actors: {
    discoverAdapters: fromPromise(async () =>
      programmingOptionsSchema.parse(await requestJson(motorIdsPath)),
    ),
    programMotor: fromPromise(
      async ({ input }: { input: ProgrammingTarget }) => {
        const result = programmedMotorSchema.parse(
          await requestJson(motorIdsPath, {
            port: input.port,
            motor_id: input.motorId,
            isolated_motor: true,
          }),
        );
        if (result.motor_id !== input.motorId) {
          throw new Error("Unexpected motor ID returned.");
        }
        return result.motor_id;
      },
    ),
  },
  guards: {
    hasProgrammingTarget: ({ context }) => {
      const { port, motor } = getWizardSelection(context);
      return Boolean(port && motor);
    },
    hasVerifiedMotor: ({ context }) => context.verifiedMotorId !== undefined,
    canSkipMotor: ({ context }) =>
      context.mode === "batch" &&
      getWizardSelection(context).motor !== undefined,
    batchComplete: ({ context }) =>
      context.mode === "batch" &&
      context.batchResults.length === context.options?.motors.length,
  },
  actions: {
    clearResult: assign({ verifiedMotorId: undefined, error: "" }),
    skipMotor: assign({
      batchResults: ({ context }): BatchResult[] => [
        ...context.batchResults,
        "skipped",
      ],
      error: "",
    }),
  },
});

const skipPendingMotor = {
  guard: "canSkipMotor",
  target: "#motor-id-wizard.interactive.awaitingConfirmation",
  actions: "skipMotor",
} as const;

export const motorIdWizard = wizard.createMachine({
  id: "motor-id-wizard",
  context: () => ({
    selectedPort: "",
    mode: "batch",
    batchResults: [],
    singleMotorId: readLastProgrammedMotorId(),
    error: "",
  }),
  initial: "refreshing",
  states: {
    refreshing: {
      entry: assign({ error: "" }),
      invoke: {
        src: "discoverAdapters",
        onDone: {
          target: "interactive",
          actions: assign({ options: ({ event }) => event.output }),
        },
        onError: {
          target: "interactive",
          actions: assign({
            options: undefined,
            error: ({ event }) => errorMessage(event.error),
          }),
        },
      },
    },
    interactive: {
      initial: "restoreCurrentStep",
      on: {
        selectMode: {
          target: ".awaitingConfirmation",
          actions: ["clearResult", assign({ mode: ({ event }) => event.mode })],
        },
        selectPort: {
          target: ".awaitingConfirmation",
          actions: [
            "clearResult",
            assign({ selectedPort: ({ event }) => event.port }),
          ],
        },
        selectMotor: {
          target: ".awaitingConfirmation",
          actions: [
            "clearResult",
            assign({ singleMotorId: ({ event }) => event.motorId }),
          ],
        },
        refreshAdapters: "#motor-id-wizard.refreshing",
        startNewBatch: {
          guard: "batchComplete",
          target: ".awaitingConfirmation",
          actions: ["clearResult", assign({ batchResults: [] })],
        },
      },
      states: {
        restoreCurrentStep: {
          always: [
            { guard: "hasVerifiedMotor", target: "verified" },
            { target: "awaitingConfirmation" },
          ],
        },
        awaitingConfirmation: {
          on: {
            confirmIsolation: {
              guard: "hasProgrammingTarget",
              target: "confirmed",
            },
            skipMotor: skipPendingMotor,
          },
        },
        confirmed: {
          on: {
            withdrawConfirmation: "awaitingConfirmation",
            programMotor: "#motor-id-wizard.programming",
            skipMotor: skipPendingMotor,
          },
        },
        verified: {
          on: {
            continueAfterVerification: {
              target: "awaitingConfirmation",
              actions: [
                assign({
                  batchResults: ({ context }) =>
                    context.mode === "batch"
                      ? [...context.batchResults, "verified"]
                      : context.batchResults,
                }),
                "clearResult",
              ],
            },
          },
        },
      },
    },
    programming: {
      entry: assign({ error: "" }),
      invoke: {
        src: "programMotor",
        input: ({ context }) => {
          const { port, motor } = getWizardSelection(context);
          if (!port || !motor)
            throw new Error("Confirmed motor or USB adapter is missing.");
          return { port, motorId: motor.id };
        },
        onDone: {
          target: "interactive.verified",
          actions: [
            ({ context, event }) => {
              if (context.mode === "single")
                rememberProgrammedMotorId(event.output);
            },
            assign({
              verifiedMotorId: ({ event }) => event.output,
              singleMotorId: ({ context, event }) =>
                context.mode === "single"
                  ? event.output
                  : context.singleMotorId,
            }),
          ],
        },
        onError: {
          target: "interactive.awaitingConfirmation",
          actions: assign({ error: ({ event }) => errorMessage(event.error) }),
        },
      },
    },
  },
});
