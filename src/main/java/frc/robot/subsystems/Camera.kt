package frc.robot.subsystems

import edu.wpi.first.cscore.OpenCvLoader
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import frc.robot.Robot
import frc.robot.lib.currentTimeToFPGA
import frc.robot.subsystems.drivetrain.Chassis
import kotlin.jvm.optionals.getOrNull
import kotlin.math.pow
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*

class Camera(val name: String, val transform: Transform3d) {
    val cam = PhotonCamera(name)
    val robotToCamera2d =
        Transform2d(transform.translation.toTranslation2d(), transform.rotation.toRotation2d())

    private val estimator =
        PhotonPoseEstimator(
            Robot.gameField,
            PoseStrategy.CONSTRAINED_SOLVEPNP,
            transform,
        )

    private val cPNPParams = ConstrainedSolvepnpParams(true, 0.5)

    // TODO guessed values, should tune one day
    private val singleTagStds: Matrix<N3, N1> = VecBuilder.fill(2.0, 2.0, 1.0)
    private val multiTagStds: Matrix<N3, N1> = VecBuilder.fill(0.1, 0.1, 0.1)

    var currentStds: Matrix<N3, N1> = singleTagStds

    private var lastEstimatedPose: EstimatedRobotPose? = null

    init {
        OpenCvLoader.forceStaticLoad()
        estimator.setMultiTagFallbackStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
    }

    fun getEstimatedGlobalPose(): EstimatedRobotPose? {
        estimator.addHeadingData(currentTimeToFPGA(Chassis.state.Timestamp), Chassis.state.Pose.rotation)
        var visionEst: EstimatedRobotPose? = null
        cam.allUnreadResults.forEach {
            visionEst = estimator.update(it, cam.cameraMatrix, cam.distCoeffs, Optional.of(cPNPParams)).getOrNull()
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
        val outOfRange =
            validTargets.size == 1 && avgDist > 2 || validTargets.size == 2 && avgDist > 5
        if (outOfRange) {
            currentStds = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
            return
        }
        // We need to improve standard deviation calculations
        currentStds.times(1 + (avgDist.pow(2) / 15))
    }
}
