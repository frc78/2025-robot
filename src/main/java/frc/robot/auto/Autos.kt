package frc.robot.auto

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.FieldGeometry
import frc.robot.lib.FieldPoses.Branch
import frc.robot.lib.command
import frc.robot.lib.meters
import frc.robot.subsystems.Intake
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.drivetrain.Chassis

object Autos {

    private fun goToLevelAndScore(level: RobotState): Command {
        return Commands.sequence(
            SuperStructure.goToScoreCoral(level),
            Commands.waitUntil { SuperStructure.atPosition }.withTimeout(3.0),
            Intake.scoreCoral,
            SuperStructure.smartGoTo(RobotState.CoralStation),
        )
    }

    private val goToCoralStationAndGetCoral by command {
        Commands.sequence(
            SuperStructure.smartGoTo(RobotState.CoralStation),
            Intake.intakeCoralThenHold().deadlineFor(Chassis.driveToClosestCenterCoralStation),
        )
    }

    @Suppress("SpreadOperator")
    val SideCoralFast by command {
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
                        Chassis.driveToPoseWithCoralOffset {
                                Chassis.state.Pose.nearest(branches.map { it.pose })
                            }
                            .alongWith(
                                Commands.sequence(
                                    // flip wrist when away from coral station with coral
                                    Commands.waitUntil {
                                            FieldGeometry.distanceToClosestLine(
                                                    FieldGeometry.CORAL_STATIONS,
                                                    Chassis.state.Pose.translation,
                                                )
                                                .meters > 1.meters && Intake.hasBranchCoral
                                        }
                                        .andThen(Wrist.goTo(RobotState.CoralStorage)),
                                    // move superstructure when close to reef
                                    Commands.waitUntil { Chassis.distanceFromPoseGoal in 0.0..1.75 }
                                        .andThen(SuperStructure.goToScoreCoral(RobotState.L4)),
                                )
                            ),
                        goToLevelAndScore(RobotState.L4),
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
                        Chassis.driveToPoseWithCoralOffset {
                                Chassis.state.Pose.nearest(branches.map { it.pose })
                            }
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
}
