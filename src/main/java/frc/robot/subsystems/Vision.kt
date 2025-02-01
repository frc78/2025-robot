package frc.robot.subsystems

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
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
                    Transform3d.struct,
                    *it.targetsUsed
                        .map {
                            it.getBestCameraToTarget()
                                .plus(
                                    Transform3d(
                                        Transform2d(
                                            Chassis.state.Pose.x,
                                            Chassis.state.Pose.y,
                                            Chassis.state.Pose.rotation,
                                        )
                                    )
                                )
                        }
                        .toTypedArray(),
                )
            }
        }
    }
}
