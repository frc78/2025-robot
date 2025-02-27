package frc.robot.auto

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.robot.subsystems.Intake
import frc.robot.subsystems.drivetrain.Chassis

fun registerNamedCommands() {
    NamedCommands.registerCommands(
        mapOf(
            "ScoreClosestLeft" to Chassis.driveToLeftBranch.andThen(PrintCommand("Scoring")),
            "WaitForCoral" to Intake.intakeCoralThenHold(),
        )
    )
}
