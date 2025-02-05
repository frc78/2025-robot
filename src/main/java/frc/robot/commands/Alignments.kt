package frc.robot.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Robot
import frc.robot.lib.calculateSpeeds
import frc.robot.subsystems.Chassis

class Alignments {
    val field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)

    fun SnapAngleToReef(): Command {
        val tagPoses = intArrayOf(22, 21, 20).map { field.getTagPose(it).get().toPose2d() }
        val pose = Chassis.state.Pose.nearest(tagPoses)

        val speeds = Robot.driveController.calculateSpeeds()
        return Chassis.applyRequest {SwerveRequest.FieldCentricFacingAngle().withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond).withTargetDirection(pose.rotation.rotateBy(Rotation2d.k180deg))}
    }
}