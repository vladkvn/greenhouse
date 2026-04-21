import { Module } from "@nestjs/common";
import { ConfigModule } from "@nestjs/config";
import { DevicesModule } from "../devices/devices.module";
import { ReadingsModule } from "../readings/readings.module";
import { MqttIngestService } from "./mqtt-ingest.service";

@Module({
  imports: [ConfigModule, DevicesModule, ReadingsModule],
  providers: [MqttIngestService],
})
export class MqttModule {}
