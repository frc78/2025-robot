package frc.robot.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Robot
import frc.robot.lib.calculateSpeeds
import frc.robot.lib.meters
import frc.robot.subsystems.Chassis

class Alignments {
    val field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
    val reefTags = intArrayOf(22, 21, 20)

    fun snapAngleToReef(): Command {
        val tagPoses = reefTags.map { field.getTagPose(it).get().toPose2d() }
        val pose = Chassis.state.Pose.nearest(tagPoses)

        val speeds = Robot.driveController.calculateSpeeds()
        return Chassis.applyRequest {SwerveRequest.FieldCentricFacingAngle().withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond).withTargetDirection(pose.rotation.rotateBy(Rotation2d.k180deg))}
    }

    fun snapToReef(relativePose: Transform2d): Command {
        val tagPoses = reefTags.map { field.getTagPose(it).get().toPose2d() }
        val tagPose = Chassis.state.Pose.nearest(tagPoses)

        val speeds = Robot.driveController.calculateSpeeds()
        return DriveToPose(tagPose.transformBy(relativePose))
    }

    fun snapToReefLeft() = snapToReef(Transform2d((-0.5).meters, (-0.5).meters, Rotation2d.kZero))
    fun snapToReefRight() = snapToReef(Transform2d((0.5).meters, (-0.5).meters, Rotation2d.kZero))
}