package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.degrees

object Wrist : SubsystemBase("Wrist") {
    fun goTo(state: RobotState): Command =
        PrintCommand("Wrist going to $state - ${state.wristAngle}")
            .alongWith(runOnce { angle = state.wristAngle })

    var angle = 0.degrees
}
