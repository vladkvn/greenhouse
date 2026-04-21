import { IsOptional, IsString, Matches, MaxLength, MinLength } from "class-validator";

export class RegisterDeviceDto {
  @IsString()
  @MinLength(1)
  @MaxLength(128)
  @Matches(/^[a-zA-Z0-9_-]+$/, {
    message: "deviceId must contain only letters, digits, underscore, hyphen",
  })
  deviceId: string;

  @IsOptional()
  @IsString()
  @MaxLength(256)
  name?: string;

  @IsOptional()
  @IsString()
  @MaxLength(64)
  firmwareVersion?: string;
}
