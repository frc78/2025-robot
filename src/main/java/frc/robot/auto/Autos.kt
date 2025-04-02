package frc.robot.auto

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.FieldGeometry
import frc.robot.lib.FieldPoses
import frc.robot.lib.FieldPoses.Branch
import frc.robot.lib.command
import frc.robot.lib.meters
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Pivot
import frc.robot.subsystems.RobotState
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
            //            SuperStructure.smartGoTo(RobotState.CoralStation), // this delays, want to
            // be faster
            Wrist.goTo(RobotState.CoralStation),
            Elevator.goTo(RobotState.CoralStation),
            Pivot.goTo(RobotState.CoralStation),
        )
    }

    private val goToCoralStationAndGetCoral by command {
        Commands.sequence(
            SuperStructure.smartGoTo(RobotState.NewCoralStation),
            Intake.intakeCoralThenHold().deadlineFor(Chassis.driveToClosestCenterCoralStation),
        )
    }

    private val alignmentDebouncer = Debouncer(0.1, Debouncer.DebounceType.kRising) // was 0.5

    @Suppress("SpreadOperator")
    val SideCoralFast by command {
        Commands.sequence(
            Intake.holdCoral
                .alongWith(Wrist.goTo(RobotState.CoralStorage))
                .alongWith(Pivot.goTo(RobotState.L4)),
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
                            .withDeadline(
                                Commands.sequence(
                                    // flip wrist and move pivot when away from coral station with
                                    // coral
                                    Commands.waitUntil {
                                        FieldGeometry.distanceToClosestLine(
                                                FieldGeometry.CORAL_STATIONS,
                                                Chassis.state.Pose.translation,
                                            )
                                            .meters >= 0.6.meters
                                    },
                                    Wrist.goTo(RobotState.CoralStorage),
                                    //                                    Pivot.goTo(RobotState.L4),
                                    //                                    Commands.waitUntil {
                                    // Chassis.isWithinGoal(2.25) },
                                    SuperStructure.goToScoreCoral(RobotState.L4),
                                    Commands.waitUntil {
                                        alignmentDebouncer.calculate(Chassis.isWithinGoal(0.05))
                                    },
                                    goToLevelAndScore(RobotState.L4),
                                )
                            ),
                        //                        goToLevelAndScore(RobotState.L4),
                        goToCoralStationAndGetCoral.withTimeout(5.0),
                    )
                }
                .toTypedArray(),
        )
    }
    @Suppress("SpreadOperator")
    val FourCoralAuto by command {
        Commands.sequence(
            Intake.intakeCoralThenHold(),
            *listOf(
                    listOf(Branch.E, Branch.I),
                    listOf(Branch.D, Branch.K),
                    listOf(Branch.C, Branch.L),
                    listOf(Branch.A, Branch.B),
                )
                .mapIndexed { i, branches ->
                    Commands.sequence(
                        Chassis.driveToPoseWithCoralOffset({
                                Chassis.state.Pose.nearest(branches.map { it.pose })
                            })
                            .alongWith(
                                Commands.waitUntil {
                                        FieldGeometry.distanceToClosestLine(
                                                FieldGeometry.CORAL_STATIONS,
                                                Chassis.state.Pose.translation,
                                            )
                                            .meters > 1.5.meters && Intake.hasBranchCoral
                                    }
                                    .andThen(Wrist.goTo(RobotState.CoralStorage))
                            ),
                        if (i == 0) goToLevelAndScore(RobotState.L4)
                        else goToLevelAndScore(RobotState.L4),
                        goToCoralStationAndGetCoral.withTimeout(5.0),
                    )
                }
                .toTypedArray(),
        )
    }

    val CenterAlgaeAuto by command {
        Commands.sequence(
            // Drive to right branch, but it's on the far side of the reef so it's swapped
            Chassis.driveToPoseWithCoralOffset({ Branch.H.pose }, true)
                .alongWith(
                    Commands.sequence(
                        Pivot.goTo(RobotState.L4),
                        Commands.waitUntil { Chassis.isWithinGoal(1.5) },
                        SuperStructure.goToScoreCoral(RobotState.L4),
                    )
                )
                .withTimeout(2.0),
            // Score L4
            goToLevelAndScore(RobotState.L4),
            getAlgaeAndScore(FieldPoses.ReefFace.GH),
            getHighAlgaeAndScore(FieldPoses.ReefFace.IJ),
            // Pathfind to EF since it's around the reef
            goToCoralStationAndGetCoral, // todo test this at WPI
            //            Chassis.pathfindToPose({ FieldPoses.ReefFace.EF.pose }, false),
            //            SuperStructure.retrieveAlgaeFromReef,
            //            Chassis.pathfindToPose({ FieldPoses.closestRightBarge }, false)
            //                .andThen(SuperStructure.autoScoreAlgaeInNet),
        )
    }

    private fun getAlgaeAndScore(face: FieldPoses.ReefFace) =
        Chassis.driveToPose({ face.pose })
            .withDeadline(
                // Get algae
                Commands.sequence(
                    Commands.waitUntil { Chassis.isWithinGoal(1.5) },
                    SuperStructure.retrieveAlgaeFromReef,
                )
            )
            .andThen(
                // Drive to barge
                Chassis.driveToBargeRightSlow
                    .withDeadline(
                        Commands.sequence(
                            Commands.waitUntil { Chassis.isWithinGoal(1.5) },
                            SuperStructure.smartGoTo(RobotState.AlgaeNet),
                            Commands.waitUntil {
                                alignmentDebouncer.calculate(Chassis.isWithinGoal(0.06))
                            },
                            SuperStructure.autoScoreAlgaeInNet,
                        )
                    )
                    .withTimeout(4.0)
            )

    private fun getHighAlgaeAndScore(face: FieldPoses.ReefFace) =
        Chassis.driveToPose({ face.pose })
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
            .andThen(
                // Drive to barge
                Chassis.driveToBargeRightSlow
                    .withDeadline(
                        Commands.sequence(
                            Commands.waitUntil { Chassis.isWithinGoal(1.5) },
                            SuperStructure.smartGoTo(RobotState.AlgaeNet),
                            Commands.waitUntil {
                                alignmentDebouncer.calculate(Chassis.isWithinGoal(0.06))
                            },
                            SuperStructure.autoScoreAlgaeInNet,
                        )
                    )
                    .withTimeout(4.0)
            )
}
