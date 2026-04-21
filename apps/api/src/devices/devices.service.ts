import { Injectable, Logger, NotFoundException } from "@nestjs/common";
import { PrismaService } from "../prisma/prisma.service";
import { RegisterDeviceDto } from "./dto/register-device.dto";

@Injectable()
export class DevicesService {
  private readonly logger = new Logger(DevicesService.name);

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
        lastKnownIp: dto.lastKnownIp,
      },
      update: {
        ...(dto.name !== undefined ? { name: dto.name } : {}),
        ...(dto.firmwareVersion !== undefined
          ? { firmwareVersion: dto.firmwareVersion }
          : {}),
        ...(dto.lastKnownIp !== undefined ? { lastKnownIp: dto.lastKnownIp } : {}),
      },
    });

    const isNew = existing === null;
    this.logger.log(
      `Device register: deviceId=${device.deviceId} isNew=${String(isNew)}` +
        (dto.name !== undefined ? ` name=${dto.name}` : "") +
        (dto.firmwareVersion !== undefined
          ? ` firmwareVersion=${dto.firmwareVersion}`
          : "") +
        (dto.lastKnownIp !== undefined ? ` lastKnownIp=${dto.lastKnownIp}` : ""),
    );

    return {
      deviceId: device.deviceId,
      registeredAt: device.registeredAt.toISOString(),
      isNew,
    };
  }

  async getByDeviceId(deviceId: string) {
    const device = await this.prisma.device.findUnique({
      where: { deviceId },
    });
    if (device === null) {
      throw new NotFoundException(`Device not found: ${deviceId}`);
    }
    return {
      deviceId: device.deviceId,
      lastKnownIp: device.lastKnownIp,
      name: device.name,
      firmwareVersion: device.firmwareVersion,
      registeredAt: device.registeredAt.toISOString(),
      lastSeenAt:
        device.lastSeenAt === null
          ? null
          : device.lastSeenAt.toISOString(),
    };
  }
}
