package frc.robot.subsystems

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.units.Units.*
import frc.robot.lib.degrees
import frc.robot.lib.meters
import org.littletonrobotics.junction.Logger

object Vision {
    private val cams: List<Camera> =
        listOf(
            Camera(
                "FL",
                Transform3d(
                    0.25.meters,
                    0.2325.meters,
                    0.0.meters,
                    Rotation3d(0.0.degrees, (90 - 61.875).degrees, (45 + (90 - 73.536)).degrees),
                ),
            ),
            Camera(
                "FR",
                Transform3d(
                    0.25.meters,
                    (-0.2325).meters,
                    0.0.meters,
                    Rotation3d(0.0.degrees, (90 - 61.875).degrees, (-45 - (90 - 73.536)).degrees),
                ),
            ),
            Camera(
                "BL",
                Transform3d(
                    (-0.25).meters,
                    0.27.meters,
                    0.0.meters,
                    Rotation3d(0.0.degrees, (90 - 61.875).degrees, (-135 - (90 - 73.536)).degrees),
                ),
            ),
            Camera(
                "BR",
                Transform3d(
                    (-0.25).meters,
                    (-0.27).meters,
                    0.0.meters,
                    Rotation3d(0.0.degrees, (90 - 61.875).degrees, (-135 - (90 - 73.536)).degrees),
                ),
            ),
        )

    fun update() {
        cams.forEach { cam ->
            cam.getEstimatedGlobalPose()?.let {
                val pose = it.estimatedPose.toPose2d()
                Chassis.addVisionMeasurement(
                    pose,
                    it.timestampSeconds,
                    cam.currentStds,
                )
                Logger.recordOutput(cam.cam.name + " est", pose)
            }
        }
    }
}
