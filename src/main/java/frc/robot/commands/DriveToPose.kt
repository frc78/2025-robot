package frc.robot.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.Alignments.headingPID
import frc.robot.subsystems.drivetrain.Chassis
import java.util.function.Supplier
import org.littletonrobotics.junction.Logger

class DriveToPose(pose: Supplier<Pose2d>) : Command() {
    private val request =
        SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
    private val xController = ProfiledPIDController(10.0, 0.0, 0.0, Constraints(1.0, 1.0))
    private val yController = ProfiledPIDController(10.0, 0.0, 0.0, Constraints(1.0, 1.0))
    private val targetPose = pose

    init {
        addRequirements(Chassis)
        request.HeadingController.setPID(headingPID.kP, headingPID.kI, headingPID.kD)
    }

    override fun initialize() {
        val state = Chassis.state
        xController.reset(state.Pose.translation.x, state.Speeds.vxMetersPerSecond)
        yController.reset(state.Pose.translation.y, state.Speeds.vyMetersPerSecond)
    }

    override fun execute() {
        val robot = Chassis.state.Pose
        val target = targetPose.get()
        Logger.recordOutput("DriveToPose target", target)
        xController.goal = TrapezoidProfile.State(target.x, 0.0)
        yController.goal = TrapezoidProfile.State(target.y, 0.0)

        Chassis.setControl(
            request
                .withVelocityX(xController.calculate(robot.translation.x))
                .withVelocityY(yController.calculate(robot.translation.y))
                .withTargetDirection(target.rotation)
        )
    }
}
