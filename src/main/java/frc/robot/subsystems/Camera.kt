package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.jvm.optionals.getOrNull
import kotlin.math.pow

class Camera (name: String, pose: Transform3d){
    // TW: We should try to avoid using abbreviated names for variables. In this case cam is pretty well understood, but keep it in mind for the future
    // TW: Kotlin lets you avoid setting the type of a variable when declaring it. It can infer the type from the value you assign to it
    // TW: This could instead just be `private val cam = PhotonCamera(name)`.
    // TW: It's usually better to not have the type because it makes it easier to change the type of the variable later
    private val cam: PhotonCamera = PhotonCamera(name)
    // TW: We could probably store the AprilTagFieldLayout in a companion object
    private val estimator: PhotonPoseEstimator = PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, pose)

    private val singleTagStds: Matrix<N3, N1> = VecBuilder.fill(4.0,4.0, 8.0)
    private val multiTagStds: Matrix<N3, N1> = VecBuilder.fill(0.5,0.5, 1.0)

    // TW: Since this is a changing variable, it might need a better name. Something like `currentStds` or `currentStdDevs`
    private var stds: Matrix<N3, N1> = singleTagStds

    init {
        estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
    }

    // TW: Maybe we split this into a variable `lastEstimatedPose`, and a function `updateEstimatedPose` that updates the variable and returns it
    // TW: Then we can call the update function more frequently if desired, but only grab the estimated pose when we need it
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
            // TW: You can use `return` here to exit earl,y and then remove the else block
            // TW: This is a small thing, but it can make the code easier to read, especially because you don't have to indent and wrap the code as much
        } else {
            // TW: tempStds doesn't need to exist. You can just set `stds` directly
            var tempStds = singleTagStds
            var nTargets = 0
            var avgDist = 0.0

            // TW: You can leverage kotlin's filter function to make this more concise
            // TW: Also, the only case that the getTagPose would be null is if the robot detects an april tag that isn't in the field layout
            // TW: I think this is unlikely enough that we _could_ ignore it, but we don't have to.
            // TW: In that case, you could use the `sumOf` function to get the total distance, and then divide by the size of the list
            targets.forEach {
                val tagPose: Pose3d? = estimator.fieldTags.getTagPose(it.fiducialId).getOrNull()
                if (tagPose != null) {
                    nTargets++
                    avgDist += tagPose.toPose2d().translation.getDistance(pose.estimatedPose.toPose2d().translation)
                }
            }

            // TW: Maybe move the 0 tags case into the first if statement with the pose == null check
            if (nTargets == 0) {
                stds = singleTagStds
            } else {
                avgDist /= nTargets

                // TW: This could be a when statement. Give it a shot!
                if (nTargets > 1) tempStds = multiTagStds
                // TW: else if
                if (nTargets == 1 && avgDist > 4) tempStds =
                    VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
                else tempStds = tempStds.times(1 + (avgDist.pow(2) / 30))
                stds = tempStds
            }
        }

    // TW: You can make the stds variable public, with a private setter
        fun getStds: Matrix<N3, N1>() {
            return stds
        }
}