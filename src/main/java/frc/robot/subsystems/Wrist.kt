package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.Subsystem

object Wrist : Subsystem {
    fun goTo(state: RobotState) = PrintCommand("Wrist going to $state - ${state.wristAngle}")
}
