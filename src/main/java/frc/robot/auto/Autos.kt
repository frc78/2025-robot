package frc.robot.auto

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.Alignments
import frc.robot.lib.command
import frc.robot.subsystems.Intake
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.drivetrain.Chassis

object Autos {
    val FourCoralAuto by command {
        val lineToIJ = PathPlannerPath.fromPathFile("AutoLine to IJ").mirrorPath()
        val ijToCoral = PathPlannerPath.fromPathFile("IJToCoral")
        Commands.sequence(
            AutoBuilder.followPath(lineToIJ),
            Chassis.driveToRightBranch,
            SuperStructure.smartGoTo(RobotState.L4),
            Commands.waitUntil{SuperStructure.a},
            Intake.scoreCoral,
            SuperStructure.smartGoTo(RobotState.CoralStation),
            Chassis.driveToClosestCoralStation,
            (Intake.intakeCoralThenHold()),
            (SuperStructure.smartGoTo(RobotState.PreScore)).alongWith(Chassis.driveToLeftBranch),
            SuperStructure.smartGoTo(RobotState.L4),
            Intake.scoreCoral,
            SuperStructure.smartGoTo(RobotState.CoralStation),
            Chassis.driveToClosestCoralStation,
            (Intake.intakeCoralThenHold()),
            (SuperStructure.smartGoTo(RobotState.PreScore)).alongWith(Chassis.driveToRightBranch),
            SuperStructure.smartGoTo(RobotState.L4),
            Intake.scoreCoral,
            SuperStructure.smartGoTo(RobotState.CoralStation),
            Chassis.driveToClosestCoralStation,
            Intake.intakeCoralThenHold(),
            (Intake.intakeCoralThenHold()),
            (SuperStructure.smartGoTo(RobotState.PreScore)).alongWith(Chassis.driveToPose { Alignments.ReefFace.AB.pose }),
            Chassis.driveToLeftBranch,
            SuperStructure.smartGoTo(RobotState.L4),
            Intake.scoreCoral,
        )
    }
    val MiddletoNet by command {
        val lineToGH = PathPlannerPath.fromPathFile("Starting to Reef")
        val ghToCoral = PathPlannerPath.fromPathFile("GHToProcessor")
        Commands.sequence(
            AutoBuilder.followPath(lineToGH),
            Chassis.driveToRightBranch,
            SuperStructure.smartGoTo(RobotState.L4),
            Intake.scoreCoral,
            SuperStructure.smartGoTo(RobotState.HighAlgaeIntake),
            Intake.intakeAlgae,
            SuperStructure.smartGoTo(RobotState.Net),
            Chassis. \\ Find Processor command idk
            Intake.

        )
    }
}
