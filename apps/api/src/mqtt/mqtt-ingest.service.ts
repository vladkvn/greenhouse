import { Injectable, Logger, OnModuleDestroy, OnModuleInit } from "@nestjs/common";
import { ConfigService } from "@nestjs/config";
import { plainToInstance } from "class-transformer";
import { validate } from "class-validator";
import * as mqtt from "mqtt";
import type { MqttClient } from "mqtt";
import { DevicesService } from "../devices/devices.service";
import { RegisterDeviceDto } from "../devices/dto/register-device.dto";
import { CreateReadingDto } from "../readings/dto/create-reading.dto";
import { ReadingsService } from "../readings/readings.service";

@Injectable()
export class MqttIngestService implements OnModuleInit, OnModuleDestroy {
  private readonly logger = new Logger(MqttIngestService.name);
  private client: MqttClient | undefined;

  constructor(
    private readonly config: ConfigService,
    private readonly devices: DevicesService,
    private readonly readings: ReadingsService,
  ) {}

  onModuleInit(): void {
    const url = this.config.get<string>("MQTT_URL");
    if (url === undefined || url.length === 0) {
      this.logger.warn("MQTT_URL not set — MQTT ingest disabled");
      return;
    }

    this.client = mqtt.connect(url);

    this.client.on("connect", () => {
      this.logger.log(`MQTT connected: ${url}`);
      void this.client?.subscribe("greenhouse/+/register", { qos: 0 });
      void this.client?.subscribe("greenhouse/+/readings", { qos: 0 });
    });

    this.client.on("message", (topic, payload) => {
      void this.handleMessage(topic, payload);
    });

    this.client.on("error", (err: Error) => {
      this.logger.error(`MQTT error: ${err.message}`);
    });

    this.client.on("close", () => {
      this.logger.warn("MQTT connection closed");
    });
  }

  onModuleDestroy(): void {
    if (this.client !== undefined) {
      this.client.end();
      this.client = undefined;
    }
  }

  private async handleMessage(topic: string, payload: Buffer): Promise<void> {
    const parts = topic.split("/");
    if (parts.length < 3 || parts[0] !== "greenhouse") {
      return;
    }
    const suffix = parts[2];
    const text = payload.toString("utf8");

    if (suffix === "register") {
      await this.handleRegister(text);
      return;
    }
    if (suffix === "readings") {
      await this.handleReadings(text);
    }
  }

  private async handleRegister(text: string): Promise<void> {
    let parsed: unknown;
    try {
      parsed = JSON.parse(text) as unknown;
    } catch {
      this.logger.warn("MQTT register: invalid JSON");
      return;
    }
    if (typeof parsed !== "object" || parsed === null) {
      return;
    }
    const dto = plainToInstance(RegisterDeviceDto, parsed, {
      enableImplicitConversion: true,
    });
    const errors = await validate(dto);
    if (errors.length > 0) {
      this.logger.warn(
        `MQTT register: validation failed: ${JSON.stringify(errors)}`,
      );
      return;
    }
    try {
      await this.devices.register(dto);
    } catch (err) {
      const msg = err instanceof Error ? err.message : String(err);
      this.logger.error(`MQTT register: ${msg}`);
    }
  }

  private async handleReadings(text: string): Promise<void> {
    let parsed: unknown;
    try {
      parsed = JSON.parse(text) as unknown;
    } catch {
      this.logger.warn("MQTT readings: invalid JSON");
      return;
    }
    if (typeof parsed !== "object" || parsed === null) {
      return;
    }
    const dto = plainToInstance(CreateReadingDto, parsed, {
      enableImplicitConversion: true,
    });
    const errors = await validate(dto);
    if (errors.length > 0) {
      this.logger.warn(
        `MQTT readings: validation failed: ${JSON.stringify(errors)}`,
      );
      return;
    }
    try {
      await this.readings.create(dto);
    } catch (err) {
      const msg = err instanceof Error ? err.message : String(err);
      this.logger.warn(`MQTT readings: ${msg}`);
    }
  }
}
