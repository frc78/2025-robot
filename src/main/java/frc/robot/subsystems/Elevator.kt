package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.Subsystem

object Elevator : Subsystem {
    fun goTo(state: RobotState) = PrintCommand("Elevator going to $state - ${state.ElevatorHeight}")
}
