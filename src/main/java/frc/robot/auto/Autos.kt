package frc.robot.auto

import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.Alignments.Branch
import frc.robot.lib.command
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
            Chassis.driveToClosestCoralStation,
            Intake.intakeCoralThenHold(),
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
}
