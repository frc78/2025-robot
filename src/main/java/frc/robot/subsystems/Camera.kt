package frc.robot.subsystems

import com.ctre.phoenix6.Utils
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Timer
import frc.robot.Robot
import frc.robot.subsystems.drivetrain.Chassis
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

    val estimator =
        PhotonPoseEstimator(
            Robot.reefOnlyField,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            transform,
        )

    // TODO guessed values, should tune one day
    private val singleTagStds: Matrix<N3, N1> = VecBuilder.fill(0.01, 0.01, 1.0)
    private val multiTagStds: Matrix<N3, N1> = VecBuilder.fill(0.00, 0.00, 0.1)
    private val outOfRangeStds =
        VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)

    var currentStds: Matrix<N3, N1> = singleTagStds
        private set

    init {
        estimator.setMultiTagFallbackStrategy(
            PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
        )
    }

    fun getEstimatedGlobalPose(): EstimatedRobotPose? {
        estimator.addHeadingData(
            Chassis.state.Timestamp - (Utils.getCurrentTimeSeconds() - Timer.getFPGATimestamp()),
            Chassis.state.Pose.rotation,
        )
        var visionEst: EstimatedRobotPose? = null
        cam.allUnreadResults.forEach { result ->
            visionEst =
                estimator.update(result).getOrNull()?.also { updateStds(it, result.targets) }
        }
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

        currentStds = if (validTargets.size > 1) multiTagStds else singleTagStds

        if (validTargets.size == 1 && avgDist > 2 && !Robot.isDisabled) {
            currentStds = outOfRangeStds
            return
        }
        // We need to improve standard deviation calculations
        currentStds *= (1 + (avgDist.pow(2) / 100)) // was / 15
    }
}
