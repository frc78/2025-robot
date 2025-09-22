package frc.robot.auto

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.commands.scoreCoralWhenClose
import frc.robot.lib.FieldPoses
import frc.robot.lib.FieldPoses.Branch
import frc.robot.lib.command
import frc.robot.lib.inches
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Pivot
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.RobotState.AlgaeNet
import frc.robot.subsystems.RobotState.CoralStorage
import frc.robot.subsystems.RobotState.L4
import frc.robot.subsystems.RobotState.NewCoralStation
import frc.robot.subsystems.RobotState.Stow
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.SuperStructure.retractWithAlgae
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.drivetrain.Chassis

object Autos {

    private fun goToLevelAndScore(level: RobotState) =
        Commands.sequence(
            SuperStructure.goToScoreCoral(level),
            Commands.waitUntil { SuperStructure.atPosition }.withTimeout(3.0),
            Intake.scoreCoral,
        )

    private val goToCoralStationAndGetCoral by command {
        Commands.sequence(
            SuperStructure.smartGoTo(NewCoralStation),
            Intake.intakeCoralThenHold.deadlineFor(Chassis.driveToClosestCenterCoralStation),
        )
    }

    @Suppress("SpreadOperator")
    val SideCoralFast by command {
        Commands.sequence(
            Intake.holdCoral.alongWith(Wrist.goTo(CoralStorage)).alongWith(Pivot.goTo(L4)),
            *listOf(
                    listOf(Branch.E, Branch.I),
                    listOf(Branch.D, Branch.K),
                    listOf(Branch.C, Branch.L),
                    listOf(Branch.A, Branch.B),
                )
                .map { branches ->
                    Commands.sequence(
                        Chassis.driveToPoseWithCoralOffset({
                                Chassis.state.Pose.nearest(branches.map { it.pose })
                            })
                            .withDeadline(scoreCoralWhenClose),
                        goToCoralStationAndGetCoral.withTimeout(5.0),
                    )
                }
                .toTypedArray(),
        )
    }

    val CenterAlgaeAuto by command {
        Commands.sequence(
            // Drive to right branch, but it's on the far side of the reef so it's swapped
            Chassis.driveToPoseWithCoralOffset { Branch.H.pose }
                .alongWith(
                    Pivot.goTo(L4)
                        .until { Chassis.isWithinGoal(1.5) }
                        .andThen(SuperStructure.goToScoreCoral(L4))
                ),
            // Score L4
            goToLevelAndScore(L4),
            SuperStructure.smartGoTo(Stow), // Added for safety, lets see if it works!
            WaitUntilCommand { Elevator.position < 15.inches }.withTimeout(0.5),
            getAlgaeAndScore(FieldPoses.ReefFace.GH),
            getHighAlgaeAndScore(FieldPoses.ReefFace.IJ),
            // Pathfind to EF since it's around the reef
            goToCoralStationAndGetCoral,
        )
    }

    private val scoreAlgaeInBarge by command {
        // Drive to barge
        Chassis.autoModeDriveToBarge.withDeadline(
            Commands.waitUntil { Chassis.isWithinGoal(1.5) }
                .andThen(SuperStructure.smartGoTo(AlgaeNet))
                .andThen(SuperStructure.autoScoreAlgaeInNet)
        )
    }

    private fun getAlgaeAndScore(face: FieldPoses.ReefFace) =
        Chassis.driveToPose { face.pose } // changed to not pathplan to save time
            .withDeadline(
                // Get algae
                Commands.waitUntil { Chassis.isWithinGoal(1.5) }
                    .andThen(SuperStructure.retrieveAlgaeFromReef)
            )
            .andThen(scoreAlgaeInBarge)

    private fun getHighAlgaeAndScore(face: FieldPoses.ReefFace) =
        Chassis.driveToPose { face.pose }
            .withDeadline(
                // Get algae
                Commands.sequence(
                    Commands.waitUntil { Chassis.isWithinGoal(0.4) },
                    Commands.sequence(
                        SuperStructure.smartGoTo(RobotState.HighAlgaeIntake)
                            .withDeadline(Intake.intakeAlgaeThenHold)
                            .andThen(retractWithAlgae())
                    ),
                )
            )
            .andThen(scoreAlgaeInBarge)
}
