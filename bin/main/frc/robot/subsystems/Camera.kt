package frc.robot.subsystems

import com.ctre.phoenix6.Utils
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.Timer
import kotlin.jvm.optionals.getOrNull
import kotlin.math.pow
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds

class Camera(name: String, pose: Transform3d) {
    val cam = PhotonCamera(name)

    companion object {
        private val field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)
    }

    // TW: We could probably store the AprilTagFieldLayout in a companion object
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

//    fun getTimeFromLastRefresh(): Duration {
//        return (lastEstimatedPose?.timestampSeconds?.minus(Timer.getFPGATimestamp())?.milliseconds) ?: Double.MAX_VALUE.milliseconds
//    }

    private fun updateStds(pose: EstimatedRobotPose?, targets: List<PhotonTrackedTarget>) {
        if (pose == null) {
            currentStds = singleTagStds
            return
        }
        var nTargets = 0
        var avgDist = 0.0

        // TW: You can leverage kotlin's filter function to make this more concise
        // TW: Also, the only case that the getTagPose would be null is if the robot detects an
        // april tag that isn't in the field layout
        // TW: I think this is unlikely enough that we _could_ ignore it, but we don't have to.
        // TW: In that case, you could use the `sumOf` function to get the total distance, and
        // then divide by the size of the list
        targets.forEach {
            val tagPose: Pose3d? = estimator.fieldTags.getTagPose(it.fiducialId).getOrNull()
            if (tagPose != null) {
                nTargets++
                avgDist +=
                    tagPose
                        .toPose2d()
                        .translation
                        .getDistance(pose.estimatedPose.toPose2d().translation)
            }
        }

        if (nTargets == 0) {
            currentStds = singleTagStds
        } else {
            avgDist /= nTargets

            if (nTargets > 1) currentStds = multiTagStds
//            if (nTargets == 1 && avgDist > 4)
//                currentStds = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
            else currentStds = currentStds.times(1 + (avgDist.pow(2) / 30))
        }
    }
}
