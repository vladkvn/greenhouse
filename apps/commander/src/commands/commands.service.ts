import {
  BadGatewayException,
  Injectable,
  Logger,
  NotFoundException,
  ServiceUnavailableException,
} from "@nestjs/common";
import { ConfigService } from "@nestjs/config";
import { SendCommandDto } from "./dto/send-command.dto";

type DeviceLookupResponse = {
  deviceId: string;
  lastKnownIp: string | null;
};

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
    const devicePort = Number(
      this.config.get<string>("DEVICE_HTTP_PORT") ?? "80",
    );
    const commandToken = this.config.get<string>("COMMAND_TOKEN") ?? "";

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

    const device = (await lookupRes.json()) as DeviceLookupResponse;

    if (
      device.lastKnownIp === null ||
      device.lastKnownIp === undefined ||
      device.lastKnownIp.length === 0
    ) {
      throw new ServiceUnavailableException(
        "Device has no lastKnownIp; ensure the module registered with IP (POST /devices/register with lastKnownIp)",
      );
    }

    let deviceUrl = `http://${device.lastKnownIp}:${String(devicePort)}/command?cmd=${encodeURIComponent(dto.cmd)}`;
    if (commandToken.length > 0) {
      deviceUrl += `&token=${encodeURIComponent(commandToken)}`;
    }

    const timeoutMs = Number(
      this.config.get<string>("DEVICE_HTTP_TIMEOUT_MS") ?? "10000",
    );
    const ac = new AbortController();
    const t = setTimeout(() => ac.abort(), timeoutMs);

    let cmdRes: Response;
    try {
      cmdRes = await fetch(deviceUrl, {
        method: "GET",
        signal: ac.signal,
      });
    } catch (err) {
      clearTimeout(t);
      const message = err instanceof Error ? err.message : String(err);
      this.logger.warn(`Device HTTP failed: ${message}`);
      throw new BadGatewayException(
        `Could not reach device at ${device.lastKnownIp}:${String(devicePort)} (${message})`,
      );
    } finally {
      clearTimeout(t);
    }

    const responseText = await cmdRes.text();
    this.logger.log(
      `Command sent: deviceId=${dto.deviceId} cmd=${dto.cmd} deviceHttp=${String(cmdRes.status)}`,
    );

    return {
      ok: cmdRes.ok,
      deviceId: dto.deviceId,
      cmd: dto.cmd,
      deviceHttpStatus: cmdRes.status,
      deviceResponse: responseText.length > 2048 ? responseText.slice(0, 2048) + "…" : responseText,
    };
  }
}
