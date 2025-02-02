package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import frc.robot.lib.degrees
import frc.robot.lib.inches
import org.littletonrobotics.junction.Logger

object Vision {
    // Measured from CAD
    private val camX = 9.486.inches
    private val camY = 10.309.inches
    private val camZ = 8.5.inches
    private val camRoll = 0.degrees
    private val camPitch = (-61.875).degrees
    private val camYaw = 30.degrees

    private val field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)

    private val cams: List<Camera> =
        listOf(
            Camera("FL", Transform3d(camX, camY, camZ, Rotation3d(camRoll, camPitch, camYaw))),
            Camera("FR", Transform3d(camX, -camY, camZ, Rotation3d(camRoll, camPitch, -camYaw))),
            Camera(
                "BL",
                Transform3d(-camX, camY, camZ, Rotation3d(camRoll, camPitch, 180.degrees - camYaw)),
            ),
            Camera(
                "BR",
                Transform3d(-camX, -camY, camZ, Rotation3d(camRoll, camPitch, 180.degrees + camYaw)),
            ),
        )

    fun update() {
        cams.forEach { cam ->
            cam.getEstimatedGlobalPose()?.let {
                val pose = it.estimatedPose.toPose2d()
                Chassis.addVisionMeasurement(pose, it.timestampSeconds, cam.currentStds)
                Logger.recordOutput(cam.cam.name + " est", pose)
                Logger.recordOutput(
                    cam.cam.name + " tags",
                    Translation3d.struct,
                    *it.targetsUsed
                        .map { field.getTagPose(it.fiducialId).get().translation }
                        .toTypedArray(),
                )
            } // ?: kotlin.run {  if (cam.getTimeFromLastRefresh() > 1000.milliseconds)
            // Logger.recordOutput(cam.cam.name + " est", Pose2d())
            //            Logger.recordOutput(cam.cam.name + " tags", Translation3d())}
        }
    }
}
