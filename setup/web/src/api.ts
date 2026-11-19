import { z } from "zod";

export const apiErrorSchema = z.object({ message: z.string() });

export async function requestJson(
  path: string,
  body?: object,
): Promise<unknown> {
  const response = await fetch(
    path,
    body && {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    },
  );
  const payload: unknown = await response.json();
  if (!response.ok) {
    const error = apiErrorSchema.safeParse(payload);
    throw new Error(
      error.success
        ? error.data.message
        : `Request failed (${response.status}).`,
    );
  }
  return payload;
}
