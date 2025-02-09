package frc.robot.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.metersPerSecond
import frc.robot.subsystems.drivetrain.Chassis
import frc.robot.subsystems.drivetrain.TunerConstants
import java.util.function.Supplier
import org.littletonrobotics.junction.Logger

fun Chassis.driveToPose(pose: Supplier<Pose2d>): Command {
    val xController =
        ProfiledPIDController(
            10.0,
            0.0,
            0.0,
            Constraints(TunerConstants.kSpeedAt12Volts.metersPerSecond, 1.0),
        )
    val yController =
        ProfiledPIDController(
            10.0,
            0.0,
            0.0,
            Constraints(TunerConstants.kSpeedAt12Volts.metersPerSecond, 1.0),
        )

    return Commands.runOnce({
            val state = Chassis.state
            xController.reset(state.Pose.translation.x, state.Speeds.vxMetersPerSecond)
            yController.reset(state.Pose.translation.y, state.Speeds.vyMetersPerSecond)
        })
        .andThen(
            applyRequest {
                val robot = Chassis.state.Pose
                val target = pose.get()
                Logger.recordOutput("DriveToPose target", target)
                xController.goal = TrapezoidProfile.State(target.x, 0.0)
                yController.goal = TrapezoidProfile.State(target.y, 0.0)

                fieldCentricFacingAngle
                    .withVelocityX(xController.calculate(robot.translation.x))
                    .withVelocityY(yController.calculate(robot.translation.y))
                    .withTargetDirection(target.rotation)
            }
        )
}
