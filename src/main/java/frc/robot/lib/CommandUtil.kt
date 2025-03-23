package frc.robot.lib

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

fun Command.andWait(until: () -> Boolean) = this.andThen(Commands.waitUntil(until))
