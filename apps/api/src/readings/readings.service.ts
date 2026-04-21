import { Injectable, NotFoundException } from "@nestjs/common";
import { PrismaService } from "../prisma/prisma.service";
import { toPrismaInputJsonObject } from "../common/json-prisma";
import { CreateReadingDto } from "./dto/create-reading.dto";

@Injectable()
export class ReadingsService {
  constructor(private readonly prisma: PrismaService) {}

  async create(dto: CreateReadingDto) {
    const device = await this.prisma.device.findUnique({
      where: { deviceId: dto.deviceId },
    });
    if (device === null) {
      throw new NotFoundException(`Device not found: ${dto.deviceId}`);
    }

    const takenAt =
      dto.takenAt !== undefined ? new Date(dto.takenAt) : undefined;

    const reading = await this.prisma.$transaction(async (tx) => {
      const created = await tx.reading.create({
        data: {
          deviceId: dto.deviceId,
          payload: toPrismaInputJsonObject(dto.payload),
          takenAt,
        },
      });

      await tx.device.update({
        where: { deviceId: dto.deviceId },
        data: { lastSeenAt: new Date() },
      });

      return created;
    });

    return {
      id: reading.id.toString(),
      receivedAt: reading.receivedAt.toISOString(),
    };
  }
}
