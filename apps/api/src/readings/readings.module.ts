import { Module } from "@nestjs/common";
import { ReadingsController } from "./readings.controller";
import { ReadingsService } from "./readings.service";

@Module({
  controllers: [ReadingsController],
  providers: [ReadingsService],
})
export class ReadingsModule {}
