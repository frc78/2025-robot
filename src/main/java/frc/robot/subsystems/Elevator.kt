package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.inches

object Elevator : SubsystemBase("Elevator") {
    fun goTo(state: RobotState): Command =
        PrintCommand("Elevator going to $state - ${state.elevatorHeight}")
            .alongWith(runOnce { height = state.elevatorHeight })

    var height = 0.inches
}
