package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*
import kotlin.jvm.optionals.getOrNull
import kotlin.math.pow

class Camera (name: String, pose: Transform3d){
    private val cam: PhotonCamera = PhotonCamera(name)
    private val estimator: PhotonPoseEstimator = PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, pose)

    private val singleTagStds: Matrix<N3, N1> = VecBuilder.fill(4.0,4.0, 8.0)
    private val multiTagStds: Matrix<N3, N1> = VecBuilder.fill(0.5,0.5, 1.0)

    private var stds: Matrix<N3, N1> = singleTagStds

    init {
        estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
    }

    fun getEstimatedGlobalPose(): EstimatedRobotPose? {
        var visionEst: EstimatedRobotPose? = null
        cam.allUnreadResults.forEach {
            visionEst = estimator.update(it).getOrNull()
            updateStds(visionEst, it.getTargets())
        }
        return visionEst //TODO does this work? It's what the example said
    }

    private fun updateStds(pose: EstimatedRobotPose?, targets: List<PhotonTrackedTarget>) {
        if (pose == null) {
            stds = singleTagStds
        } else {
            var tempStds = singleTagStds
            var nTargets = 0
            var avgDist = 0.0

            targets.forEach {
                val tagPose: Pose3d? = estimator.fieldTags.getTagPose(it.fiducialId).getOrNull()
                if (tagPose != null) {
                    nTargets++
                    avgDist += tagPose.toPose2d().translation.getDistance(pose.estimatedPose.toPose2d().translation)
                }
            }

            if(nTargets == 0) {
                stds = singleTagStds
            } else {
                avgDist /= nTargets

                if(nTargets > 1) tempStds = multiTagStds
                if(nTargets == 1 && avgDist > 4) tempStds = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
                else tempStds = tempStds.times(1 + (avgDist.pow(2) / 30))
                stds = tempStds
            }
        }

        fun getStds: Matrix<N3, N1>() {
            return stds
        }
    }
}