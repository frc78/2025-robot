package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator

object Vision {
    val cams: List<Camera> = listOf(
        Camera("LU", Transform3d()),
        Camera("RU", Transform3d()))
}
