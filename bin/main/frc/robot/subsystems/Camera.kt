package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator

class Camera (name: String, pose: Transform3d){
    val cam: PhotonCamera = PhotonCamera(name)
    val estimator: PhotonPoseEstimator = PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, pose)

    fun getEstimatedPose():  {
        return estimator.update(cam.)
    }
}