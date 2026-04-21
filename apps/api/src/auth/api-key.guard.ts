import {
  CanActivate,
  ExecutionContext,
  Injectable,
  UnauthorizedException,
} from "@nestjs/common";
import { ConfigService } from "@nestjs/config";

@Injectable()
export class ApiKeyGuard implements CanActivate {
  constructor(private readonly config: ConfigService) {}

  canActivate(context: ExecutionContext): boolean {
    const expected = this.config.get<string>("API_KEY");
    if (expected === undefined || expected.length === 0) {
      throw new UnauthorizedException("API_KEY is not configured");
    }

    const request = context.switchToHttp().getRequest<{
      headers: Record<string, string | string[] | undefined>;
    }>();
    const raw = request.headers["x-api-key"];
    const received =
      typeof raw === "string" ? raw : Array.isArray(raw) ? raw[0] : undefined;

    if (received !== expected) {
      throw new UnauthorizedException("Invalid or missing X-API-Key");
    }

    return true;
  }
}
