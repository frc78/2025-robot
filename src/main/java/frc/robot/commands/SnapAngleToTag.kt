package frc.robot.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.lib.REEF_POSITION
import frc.robot.subsystems.Chassis

class SnapAngleToTag: Command() {
    val request = SwerveRequest.ApplyFieldSpeeds()
    val PIDController = ProfiledPIDController(1.0, 0.0, 0.0, Constraints(1.0, 1.0))

    var targetPose: Pose2d = Pose2d()

    init {
        addRequirements(Chassis)
    }

    override fun execute() {
        val robot = Chassis.state.Pose

        // Theoretically gets us vector from robot to reef
        val robotToReef = robot.translation.minus(REEF_POSITION)

        Chassis.applyRequest {
            with(request) {
                withSpeeds(ChassisSpeeds(0.0, 0.0, PIDController.calculate()))
            } }
    }
}
