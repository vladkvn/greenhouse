import { IsIn, IsString, Matches, MaxLength, MinLength } from "class-validator";

export class SendCommandDto {
  @IsString()
  @MinLength(1)
  @MaxLength(128)
  @Matches(/^[a-zA-Z0-9_-]+$/, {
    message: "deviceId must contain only letters, digits, underscore, hyphen",
  })
  deviceId: string;

  @IsString()
  @IsIn([
    "PING",
    "RELAY_ON",
    "RELAY_OFF",
    "VENT_OPEN",
    "VENT_CLOSE",
  ])
  cmd: string;
}
