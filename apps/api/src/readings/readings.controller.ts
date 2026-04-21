import { Body, Controller, Post } from "@nestjs/common";
import { CreateReadingDto } from "./dto/create-reading.dto";
import { ReadingsService } from "./readings.service";

@Controller("readings")
export class ReadingsController {
  constructor(private readonly readings: ReadingsService) {}

  @Post()
  create(@Body() body: CreateReadingDto) {
    return this.readings.create(body);
  }
}
