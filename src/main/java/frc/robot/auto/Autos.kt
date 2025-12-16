package frc.robot.auto

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.commands.scoreCoralWhenClose
import frc.robot.lib.FieldPoses
import frc.robot.lib.FieldPoses.Branch
import frc.robot.lib.command
import frc.robot.lib.inches
import frc.robot.lib.meters
import frc.robot.subsystems.*
import frc.robot.subsystems.RobotState.*
import frc.robot.subsystems.SuperStructure.retractWithAlgae
import frc.robot.subsystems.drivetrain.Chassis

object Autos {

    private fun goToLevelAndScore(level: RobotState): Command {
        return sequence(
            SuperStructure.goToScoreCoral(level),
            waitUntil { SuperStructure.atPosition }.withTimeout(3.0),
            Intake.scoreCoral,
            Wrist.goTo(CoralStation),
            Elevator.goTo(CoralStation),
            Pivot.goTo(CoralStation),
        )
    }

    private val goToCoralStationAndGetCoral by command {
        sequence(
                Intake.intakeCoral,
                SuperStructure.smartGoTo(NewCoralStation),
                Chassis.driveToClosestCenterCoralStation,
            )
            .until { Intake.holdingCoral }
    }

    @Suppress("SpreadOperator")
    val SideCoralFast by command {
        sequence(
            Intake.intakeCoral.alongWith(Wrist.goTo(CoralStorage), Pivot.goTo(L4)),
            *listOf(
                    listOf(Branch.E, Branch.I),
                    listOf(Branch.D, Branch.K),
                    listOf(Branch.C, Branch.L),
                    listOf(Branch.A, Branch.B),
                )
                .map { branches ->
                    sequence(
                        Chassis.driveToPoseWithCoralOffset {
                                Chassis.state.Pose.nearest(branches.map { it.pose })
                            }
                            .withDeadline(
                                // Flip to scoring position after leaving coral station
                                Commands.waitSeconds(0.5)
                                    .andThen(
                                        Wrist.goTo(CoralStorage),
                                        Pivot.goTo(L4),
                                        scoreCoralWhenClose,
                                    )
                            ),
                        goToCoralStationAndGetCoral,
                    )
                }
                .toTypedArray(),
        )
    }

    @Suppress("SpreadOperator")
    val OPSideCoral by command {
        sequence(
            Intake.intakeCoral.alongWith(Wrist.goTo(CoralStorage)).alongWith(Pivot.goTo(L4)),
            *listOf(
                    listOf(Branch.F, Branch.I),
                    listOf(Branch.C, Branch.L),
                    listOf(Branch.D, Branch.K),
                    listOf(Branch.E, Branch.J),
                )
                .mapIndexed { index, branches ->
                    sequence(
                        (if (index == 3)
                                Chassis.pathplanToPose(approachDistance = 0.1.meters) {
                                    Chassis.state.Pose.nearest(branches.map { it.pose })
                                        .transformBy(
                                            Transform2d(
                                                0.0.inches,
                                                -Intake.coralLocation,
                                                Rotation2d.kZero,
                                            )
                                        )
                                }
                            else
                                Chassis.driveToPoseWithCoralOffset({
                                    Chassis.state.Pose.nearest(branches.map { it.pose })
                                }))
                            .withDeadline(scoreCoralWhenClose),
                        goToCoralStationAndGetCoral.withTimeout(5.0),
                    )
                }
                .toTypedArray(),
        )
    }

    val CenterAlgaeAuto by command {
        sequence(
            // Drive to right branch, but it's on the far side of the reef so it's swapped
            Chassis.driveToPoseWithCoralOffset { Branch.H.pose }
                .alongWith(
                    sequence(
                        Pivot.goTo(L4),
                        waitUntil { Chassis.isWithinGoal(1.5) },
                        SuperStructure.goToScoreCoral(L4),
                    )
                )
                .withTimeout(2.0),
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
            waitUntil { Chassis.isWithinGoal(1.5) }
                .andThen(SuperStructure.smartGoTo(AlgaeNet))
                .andThen(SuperStructure.autoScoreAlgaeInNet)
        )
    }

    private fun getAlgaeAndScore(face: FieldPoses.ReefFace) =
        Chassis.driveToPose { face.pose } // changed to not pathplan to save time
            .withDeadline(
                // Get algae
                waitUntil { Chassis.isWithinGoal(1.5) }
                    .andThen(SuperStructure.retrieveAlgaeFromReef)
            )
            .andThen(scoreAlgaeInBarge)

    private fun getHighAlgaeAndScore(face: FieldPoses.ReefFace) =
        Chassis.driveToPose { face.pose }
            .withDeadline(
                // Get algae
                sequence(
                    Intake.intakeAlgae,
                    waitUntil { Chassis.isWithinGoal(0.4) },
                    sequence(
                        SuperStructure.smartGoTo(HighAlgaeIntake),
                        waitUntil { Intake.holdingAlgae },
                        retractWithAlgae(),
                    ),
                )
            )
            .andThen(scoreAlgaeInBarge)
}
