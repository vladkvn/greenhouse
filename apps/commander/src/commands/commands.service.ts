import {
  BadGatewayException,
  Injectable,
  Logger,
  NotFoundException,
} from "@nestjs/common";
import { ConfigService } from "@nestjs/config";
import * as mqtt from "mqtt";
import { SendCommandDto } from "./dto/send-command.dto";

@Injectable()
export class CommandsService {
  private readonly logger = new Logger(CommandsService.name);

  constructor(private readonly config: ConfigService) {}

  async sendToDevice(dto: SendCommandDto) {
    const apiBase = this.config.getOrThrow<string>("GREENHOUSE_API_URL").replace(
      /\/$/,
      "",
    );
    const apiKey = this.config.getOrThrow<string>("GREENHOUSE_API_KEY");

    const lookupUrl = `${apiBase}/devices/by-id/${encodeURIComponent(dto.deviceId)}`;
    const lookupRes = await fetch(lookupUrl, {
      headers: { "X-API-Key": apiKey },
    });

    if (lookupRes.status === 404) {
      throw new NotFoundException(`Device not found: ${dto.deviceId}`);
    }

    if (!lookupRes.ok) {
      const body = await lookupRes.text();
      this.logger.warn(
        `Greenhouse API lookup failed: ${String(lookupRes.status)} ${body}`,
      );
      throw new BadGatewayException(
        `Greenhouse API returned ${String(lookupRes.status)}`,
      );
    }

    const mqttUrl = this.config.getOrThrow<string>("MQTT_URL");
    const topic = `greenhouse/${dto.deviceId}/cmd`;

    await new Promise<void>((resolve, reject) => {
      const client = mqtt.connect(mqttUrl);
      const timeoutMs = Number(
        this.config.get<string>("MQTT_PUBLISH_TIMEOUT_MS") ?? "8000",
      );
      const timer = setTimeout(() => {
        client.end(true);
        reject(new Error("MQTT publish timeout"));
      }, timeoutMs);

      client.on("error", (err: Error) => {
        clearTimeout(timer);
        client.end(true);
        reject(err);
      });

      client.on("connect", () => {
        client.publish(topic, dto.cmd, { qos: 0 }, (err) => {
          clearTimeout(timer);
          client.end();
          if (err !== undefined && err !== null) {
            reject(err);
          } else {
            resolve();
          }
        });
      });
    }).catch((err: unknown) => {
      const message = err instanceof Error ? err.message : String(err);
      this.logger.warn(`MQTT publish failed: ${message}`);
      throw new BadGatewayException(`MQTT: ${message}`);
    });

    this.logger.log(
      `Command published via MQTT: deviceId=${dto.deviceId} cmd=${dto.cmd} topic=${topic}`,
    );

    return {
      ok: true,
      deviceId: dto.deviceId,
      cmd: dto.cmd,
      transport: "mqtt" as const,
      topic,
    };
  }
}
