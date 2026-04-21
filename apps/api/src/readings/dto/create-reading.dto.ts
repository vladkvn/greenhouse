import { Type } from "class-transformer";
import {
  IsISO8601,
  IsObject,
  IsOptional,
  IsString,
  Matches,
  MaxLength,
  MinLength,
  Validate,
  ValidatorConstraint,
  ValidatorConstraintInterface,
  ValidationArguments,
} from "class-validator";

function isJsonPrimitive(
  v: unknown,
): v is string | number | boolean | null {
  if (v === null) {
    return true;
  }
  const t = typeof v;
  return t === "string" || t === "number" || t === "boolean";
}

function isSensorPayloadValue(v: unknown): boolean {
  if (isJsonPrimitive(v)) {
    return true;
  }
  if (Array.isArray(v)) {
    return v.every((item) => isSensorPayloadValue(item));
  }
  if (typeof v === "object" && v !== null) {
    return Object.values(v).every((item) => isSensorPayloadValue(item));
  }
  return false;
}

@ValidatorConstraint({ name: "isSensorPayloadObject", async: false })
class IsSensorPayloadObjectConstraint implements ValidatorConstraintInterface {
  validate(value: unknown, _args: ValidationArguments): boolean {
    if (typeof value !== "object" || value === null || Array.isArray(value)) {
      return false;
    }
    return Object.values(value).every((v) => isSensorPayloadValue(v));
  }

  defaultMessage(): string {
    return "payload must be a JSON object with string, number, boolean, null, or nested object/array values";
  }
}

export class CreateReadingDto {
  @IsString()
  @MinLength(1)
  @MaxLength(128)
  @Matches(/^[a-zA-Z0-9_-]+$/, {
    message: "deviceId must contain only letters, digits, underscore, hyphen",
  })
  deviceId: string;

  @IsObject()
  @Validate(IsSensorPayloadObjectConstraint)
  @Type(() => Object)
  payload: Record<string, unknown>;

  @IsOptional()
  @IsISO8601()
  takenAt?: string;
}
