package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import kotlin.jvm.optionals.getOrNull
import kotlin.math.pow
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget

class Camera(val name: String, val pose: Transform3d) {
    val cam = PhotonCamera(name)
    val robotToCamera2d =
        Transform2d(pose.translation.toTranslation2d(), pose.rotation.toRotation2d())

    companion object {
        private val field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
    }

    private val estimator =
        PhotonPoseEstimator(
            field,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            pose,
        )

    // TODO guessed values, should tune one day
    private val singleTagStds: Matrix<N3, N1> = VecBuilder.fill(0.5, 0.5, 1.0)
    private val multiTagStds: Matrix<N3, N1> = VecBuilder.fill(0.3, 0.3, 0.5)

    var currentStds: Matrix<N3, N1> = singleTagStds
        private set

    private var lastEstimatedPose: EstimatedRobotPose? = null

    init {
        estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
    }

    fun getEstimatedGlobalPose(): EstimatedRobotPose? {
        var visionEst: EstimatedRobotPose? = null
        cam.allUnreadResults.forEach {
            visionEst = estimator.update(it).getOrNull()
            updateStds(visionEst, it.getTargets())
        }
        lastEstimatedPose = visionEst
        return visionEst // TODO does this work? It's what the example said
    }

    private fun updateStds(pose: EstimatedRobotPose?, targets: List<PhotonTrackedTarget>) {
        if (pose == null) {
            currentStds = singleTagStds
            return
        }

        val validTargets =
            targets.mapNotNull { estimator.fieldTags.getTagPose(it.fiducialId).getOrNull() }
        val totalDistance =
            validTargets.sumOf { it.translation.getDistance(pose.estimatedPose.translation) }

        if (validTargets.isEmpty()) {
            currentStds = singleTagStds
        } else {
            val avgDist = totalDistance / validTargets.size

            if (validTargets.size > 1) currentStds = multiTagStds

            currentStds =
                if (validTargets.size == 1 && avgDist > 4)
                    VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
                else currentStds.times(1 + (avgDist.pow(2) / 30))
        }
    }
}
