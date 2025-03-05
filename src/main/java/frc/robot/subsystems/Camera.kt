package frc.robot.subsystems

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import frc.robot.Robot
import kotlin.jvm.optionals.getOrNull
import kotlin.math.pow
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget

class Camera(val name: String, val transform: Transform3d) {
    val cam = PhotonCamera(name)
    val robotToCamera2d =
        Transform2d(transform.translation.toTranslation2d(), transform.rotation.toRotation2d())

    private val estimator =
        PhotonPoseEstimator(
            Robot.gameField,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            transform,
        )

    // TODO guessed values, should tune one day
    private val singleTagStds: Matrix<N3, N1> = VecBuilder.fill(2.0, 2.0, 1.0)
    private val multiTagStds: Matrix<N3, N1> = VecBuilder.fill(0.5, 0.5, 0.5)

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
            return
        }
        val avgDist = totalDistance / validTargets.size

        if (validTargets.size > 1) currentStds = multiTagStds
        if (validTargets.size == 1 && avgDist > 2) {
            currentStds = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
            return
        }
        // We need to improve standard deviation calculations
        currentStds.times(1 + (avgDist.pow(2) / 15))
    }
}
