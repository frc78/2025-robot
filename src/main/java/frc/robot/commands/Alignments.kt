package frc.robot.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import com.pathplanner.lib.config.PIDConstants
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Robot
import frc.robot.lib.calculateSpeeds
import frc.robot.lib.meters
import frc.robot.subsystems.drivetrain.Chassis
import java.util.function.Supplier

object Alignments {
    val headingPID = PIDConstants(5.0, 0.0, 0.0)

    private val field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
    private val reefPoses
        get() =
            (if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                    intArrayOf(17, 18, 19, 20, 21, 22)
                } else {
                    intArrayOf(6, 7, 8, 9, 10, 11)
                })
                .map { field.getTagPose(it).get().toPose2d() }

    private val snapAngleRequest =
        SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)

    init {
        snapAngleRequest.HeadingController.setPID(headingPID.kP, headingPID.kI, headingPID.kD)
    }

    private fun snapToReef(relativePose: Supplier<Transform2d>): Command {
        return DriveToPose(
            ({ Chassis.state.Pose.nearest(reefPoses).transformBy(relativePose.get()) })
        )
    }

    fun snapAngleToReef(): Command {
        return Chassis.applyRequest {
            val pose = Chassis.state.Pose.nearest(reefPoses)
            val speeds = Robot.driveController.hid.calculateSpeeds()

            snapAngleRequest
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withTargetDirection(pose.rotation)
        }
    }

    fun snapToReefLeft() = snapToReef { Transform2d((0.4).meters, (-0.3).meters, Rotation2d.kZero) }

    fun snapToReefRight() = snapToReef { Transform2d((0.4).meters, (0.3).meters, Rotation2d.kZero) }
}
