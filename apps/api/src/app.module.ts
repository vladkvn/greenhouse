import { Module } from "@nestjs/common";
import { ConfigModule } from "@nestjs/config";
import { APP_GUARD } from "@nestjs/core";
import { ApiKeyGuard } from "./auth/api-key.guard";
import { DevicesModule } from "./devices/devices.module";
import { MqttModule } from "./mqtt/mqtt.module";
import { PrismaModule } from "./prisma/prisma.module";
import { ReadingsModule } from "./readings/readings.module";

@Module({
  imports: [
    ConfigModule.forRoot({
      isGlobal: true,
      envFilePath: [".env.local", ".env"],
    }),
    PrismaModule,
    DevicesModule,
    ReadingsModule,
    MqttModule,
  ],
  providers: [
    {
      provide: APP_GUARD,
      useClass: ApiKeyGuard,
    },
  ],
})
export class AppModule {}
