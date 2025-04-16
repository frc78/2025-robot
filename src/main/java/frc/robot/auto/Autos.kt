package frc.robot.auto

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import frc.robot.commands.scoreCoralWhenClose
import frc.robot.lib.FieldPoses
import frc.robot.lib.FieldPoses.Branch
import frc.robot.lib.Level
import frc.robot.lib.ScoreSelector.SelectedLevel
import frc.robot.lib.command
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Pivot
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.RobotState.AlgaeNet
import frc.robot.subsystems.RobotState.CoralStation
import frc.robot.subsystems.RobotState.CoralStorage
import frc.robot.subsystems.RobotState.L4
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.SuperStructure.retractWithAlgae
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.drivetrain.Chassis

object Autos {

    private fun goToLevelAndScore(level: RobotState): Command {
        return Commands.sequence(
            SuperStructure.goToScoreCoral(level),
            Commands.waitUntil { SuperStructure.atPosition }.withTimeout(3.0),
            Intake.scoreCoral,
            Wrist.goTo(CoralStation),
            Elevator.goTo(CoralStation),
            Pivot.goTo(CoralStation),
        )
    }

    private val goToCoralStationAndGetCoral by command {
        Intake.intakeCoralThenHold()
            .deadlineFor(
                Chassis.driveToClosestCenterCoralStation.alongWith(
                    Commands.waitUntil { SuperStructure.atPosition }
                        .andThen(SuperStructure.smartGoTo(RobotState.NewCoralStation))
                )
            )
    }

    @Suppress("SpreadOperator")
    val SideCoralFast by command {
        Commands.sequence(
            runOnce({ SelectedLevel = Level.L4 }),
            Intake.holdCoral.alongWith(Wrist.goTo(CoralStorage)).alongWith(Pivot.goTo(L4)),
            *listOf(
                    listOf(Branch.E, Branch.I),
                    listOf(Branch.A, Branch.B),
                    listOf(Branch.D, Branch.K),
                    listOf(Branch.C, Branch.L),
                )
                .map { branches ->
                    Commands.sequence(
                        Chassis.pathplanToPoseWithCoralOffset {
                                Chassis.state.Pose.nearest(branches.map { it.pose })
                            }
                            .withDeadline(
                                scoreCoralWhenClose.andThen(SuperStructure.smartGoTo(CoralStorage))
                            ),
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
                    Commands.sequence(
                        Pivot.goTo(L4),
                        Commands.waitUntil { Chassis.isWithinGoal(1.5) },
                        SuperStructure.goToScoreCoral(L4),
                    )
                )
                .until { Chassis.isWithinGoal(0.05) },
            // Score L4
            goToLevelAndScore(L4),
            SuperStructure.smartGoTo(CoralStorage), // Added for safety, lets see if it works!
            getAlgaeAndScore(FieldPoses.ReefFace.GH),
            getHighAlgaeAndScore(FieldPoses.ReefFace.IJ),
            getAlgaeAndScore(FieldPoses.ReefFace.EF),
        )
    }

    val JackInTheBotKiller by command {
        Commands.sequence(
            // Drive to right branch, but it's on the far side of the reef so it's swapped
            Chassis.driveToPoseWithCoralOffset { Branch.H.pose }
                .alongWith(
                    Commands.sequence(
                        Pivot.goTo(L4),
                        Commands.waitUntil { Chassis.isWithinGoal(1.5) },
                        SuperStructure.goToScoreCoral(L4),
                    )
                )
                .until { Chassis.isWithinGoal(0.05) },
            // Score L4
            goToLevelAndScore(L4),
            SuperStructure.smartGoTo(CoralStorage), // Added for safety, lets see if it works!
            getAlgaeAndScore(FieldPoses.ReefFace.GH),
            getHighAlgaeAndScore(FieldPoses.ReefFace.GH, true),
            getAlgaeAndScore(FieldPoses.ReefFace.EF, true),
        )
    }

    private val scoreAlgaeInBarge by command {
        // Drive to barge
        Chassis.driveToBargeRight
            .alongWith(
                Commands.waitUntil { Chassis.isWithinGoal(1.5) }
                    .andThen(SuperStructure.smartGoTo(AlgaeNet))
            )
            .andThen(SuperStructure.autoScoreAlgaeInNet)
    }

    private fun getAlgaeAndScore(face: FieldPoses.ReefFace, opponent: Boolean = false) =
        Chassis.pathplanToPose { if (opponent) face.opponentPose else face.pose }
            .withDeadline(
                // Get algae
                // It will be going to coralStorage, after which we can go to pose
                Commands.waitUntil { SuperStructure.atPosition }
                    .andThen(SuperStructure.retrieveAlgaeFromReef)
            )
            .andThen(scoreAlgaeInBarge)

    private fun getHighAlgaeAndScore(face: FieldPoses.ReefFace, opponent: Boolean = false) =
        Chassis.pathplanToPose { if (opponent) face.opponentPose else face.pose }
            .withDeadline(
                // Get algae
                Commands.sequence(
                    Commands.waitUntil { Chassis.isWithinGoal(0.3) },
                    Commands.sequence(
                        SuperStructure.smartGoTo(RobotState.HighAlgaeIntake)
                            .withDeadline(Intake.intakeAlgaeThenHold())
                            .andThen(retractWithAlgae())
                    ),
                )
            )
            .andThen(scoreAlgaeInBarge)
}
