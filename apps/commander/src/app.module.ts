import { Module } from "@nestjs/common";
import { ConfigModule } from "@nestjs/config";
import { APP_GUARD } from "@nestjs/core";
import { CommanderApiKeyGuard } from "./auth/commander-api-key.guard";
import { CommandsModule } from "./commands/commands.module";

@Module({
  imports: [
    ConfigModule.forRoot({
      isGlobal: true,
      envFilePath: [".env.local", ".env"],
    }),
    CommandsModule,
  ],
  providers: [
    {
      provide: APP_GUARD,
      useClass: CommanderApiKeyGuard,
    },
  ],
})
export class AppModule {}
