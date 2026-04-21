import { Body, Controller, Post } from "@nestjs/common";
import { DevicesService } from "./devices.service";
import { RegisterDeviceDto } from "./dto/register-device.dto";

@Controller("devices")
export class DevicesController {
  constructor(private readonly devices: DevicesService) {}

  @Post("register")
  register(@Body() body: RegisterDeviceDto) {
    return this.devices.register(body);
  }
}
