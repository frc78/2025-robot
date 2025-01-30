package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.Subsystem

object Pivot : Subsystem {

    fun goTo(state: RobotState) = PrintCommand("Pivot going to $state - ${state.pivotAngle}")
}
