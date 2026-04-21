import { Body, Controller, Post } from "@nestjs/common";
import { CommandsService } from "./commands.service";
import { SendCommandDto } from "./dto/send-command.dto";

@Controller("commands")
export class CommandsController {
  constructor(private readonly commands: CommandsService) {}

  @Post()
  send(@Body() body: SendCommandDto) {
    return this.commands.sendToDevice(body);
  }
}
