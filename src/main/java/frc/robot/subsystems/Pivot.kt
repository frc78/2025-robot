package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.degrees

object Pivot : SubsystemBase("Pivot") {
    fun goTo(state: RobotState): Command =
        PrintCommand("Pivot going to $state - ${state.pivotAngle}")
            .alongWith(runOnce { angle = state.pivotAngle })

    var angle = 45.degrees
}
