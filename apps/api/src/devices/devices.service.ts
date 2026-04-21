import { Injectable } from "@nestjs/common";
import { PrismaService } from "../prisma/prisma.service";
import { RegisterDeviceDto } from "./dto/register-device.dto";

@Injectable()
export class DevicesService {
  constructor(private readonly prisma: PrismaService) {}

  async register(dto: RegisterDeviceDto) {
    const existing = await this.prisma.device.findUnique({
      where: { deviceId: dto.deviceId },
    });

    const device = await this.prisma.device.upsert({
      where: { deviceId: dto.deviceId },
      create: {
        deviceId: dto.deviceId,
        name: dto.name,
        firmwareVersion: dto.firmwareVersion,
      },
      update: {
        ...(dto.name !== undefined ? { name: dto.name } : {}),
        ...(dto.firmwareVersion !== undefined
          ? { firmwareVersion: dto.firmwareVersion }
          : {}),
      },
    });

    return {
      deviceId: device.deviceId,
      registeredAt: device.registeredAt.toISOString(),
      isNew: existing === null,
    };
  }
}
