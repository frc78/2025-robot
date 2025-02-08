package frc.robot.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.drivetrain.Chassis
import java.util.function.Supplier

class DriveToPose(pose: Supplier<Pose2d>) : Command() {
    private val request = SwerveRequest.FieldCentricFacingAngle()
    private val xController = ProfiledPIDController(10.0, 0.0, 0.0, Constraints(1.0, 1.0))
    private val yController = ProfiledPIDController(10.0, 0.0, 0.0, Constraints(1.0, 1.0))
    private val targetPose = pose

    init {
        addRequirements(Chassis)
    }

    override fun initialize() {
        val state = Chassis.state
        xController.reset(state.Pose.translation.x, state.Speeds.vxMetersPerSecond)
        yController.reset(state.Pose.translation.y, state.Speeds.vyMetersPerSecond)
    }

    override fun execute() {
        val robot = Chassis.state.Pose
        val target = targetPose.get()

        Chassis.setControl(
            request
                .withVelocityX(xController.calculate(robot.translation.x, target.x))
                .withVelocityY(yController.calculate(robot.translation.y, target.y))
                .withTargetDirection(target.rotation)
        )
    }
}
