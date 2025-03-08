package frc.robot.auto

import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.FieldPoses
import frc.robot.lib.FieldPoses.Branch
import frc.robot.lib.command
import frc.robot.lib.inchesPerSecond
import frc.robot.lib.seconds
import frc.robot.subsystems.Intake
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.drivetrain.Chassis

object Autos {

    private val goToL4AndScore by command {
        Commands.sequence(
            SuperStructure.goToScoreCoral(RobotState.L4),
            Commands.waitUntil { SuperStructure.atPosition },
            Intake.scoreCoral,
            SuperStructure.smartGoTo(RobotState.CoralStation),
        )
    }

    private val goToCoralStationAndGetCoral by command {
        Commands.sequence(
            SuperStructure.smartGoTo(RobotState.CoralStation),
            Chassis.snapToClosestSubstation({ 0.0 }, { withVelocityX(1.inchesPerSecond) }),
            Intake.intakeCoralThenHold(),
        )
    }

    private val getAlgaeFromL3 by command {
        Commands.sequence(
            SuperStructure.smartGoTo(RobotState.HighAlgaeIntake),
            Intake.intakeAlgaeThenHold()
                .deadlineFor(
                    Chassis.robotCentricDrive { withVelocityX(1.inchesPerSecond) }
                        .withTimeout(5.seconds)
                ),
            SuperStructure.smartGoTo(RobotState.CoralStation),
        )
    }
    @Suppress("SpreadOperator")
    val FourCoralAuto by command {
        Commands.sequence(
            Intake.intakeCoralThenHold(),
            *listOf(
                    listOf(Branch.E, Branch.J),
                    listOf(Branch.D, Branch.K),
                    listOf(Branch.C, Branch.L),
                    listOf(Branch.A, Branch.B),
                )
                .map {
                    Commands.sequence(
                        Chassis.driveToPoseWithCoralOffset {
                            Chassis.state.Pose.nearest(it.map { it.pose })
                        },
                        goToL4AndScore,
                        goToCoralStationAndGetCoral,
                    )
                }
                .toTypedArray(),
        )
    }

    val CenterAlgae by command {
        Commands.sequence(
            Chassis.driveToPose { Branch.H.pose },
            goToL4AndScore,
            Chassis.driveToPose { FieldPoses.ReefFace.GH.pose },
            getAlgaeFromL3,
            Chassis.snapToBarge({ 0.0 }, { this }),
            SuperStructure.smartGoTo(RobotState.AlgaeNet),
            Intake.scoreAlgae,
            SuperStructure.smartGoTo(RobotState.CoralStation),
            Chassis.driveToPose { FieldPoses.ReefFace.GH.pose },
        )
    }
}
