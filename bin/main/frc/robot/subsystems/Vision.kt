package frc.robot.subsystems

import edu.wpi.first.math.geometry.Transform3d
import frc.robot.subsystems.Chassis.addVisionMeasurement

object Vision {
    private val cams: List<Camera> =
        listOf(
            Camera("FL", Transform3d()),
            Camera("FR", Transform3d()),
            Camera("BL", Transform3d()),
        )

    fun update() {
        cams.forEach { cam ->
            cam.getEstimatedGlobalPose()?.let {
                Chassis.addVisionMeasurement(
                    it.estimatedPose.toPose2d(),
                    it.timestampSeconds,
                    cam.currentStds,
                )

                
            }
        }
    }
}
