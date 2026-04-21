import { Prisma } from "@prisma/client";

function toInputJsonValueOrNull(value: unknown): Prisma.InputJsonValue | null {
  if (value === null) {
    return null;
  }
  if (typeof value === "string") {
    return value;
  }
  if (typeof value === "number") {
    return value;
  }
  if (typeof value === "boolean") {
    return value;
  }
  if (Array.isArray(value)) {
    return value.map((item) => toInputJsonValueOrNull(item));
  }
  if (typeof value === "object") {
    return buildInputJsonObject(value);
  }
  throw new Error("Unsupported JSON value for Prisma");
}

function buildInputJsonObject(value: object): Prisma.InputJsonObject {
  if (Array.isArray(value)) {
    throw new Error("Expected plain object");
  }
  const out: Record<string, Prisma.InputJsonValue | null> = {};
  for (const [k, v] of Object.entries(value)) {
    out[k] = toInputJsonValueOrNull(v);
  }
  return out satisfies Prisma.InputJsonObject;
}

export function toPrismaInputJsonObject(
  value: Record<string, unknown>,
): Prisma.InputJsonObject {
  return buildInputJsonObject(value);
}
