package frc.robot.lib

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.XboxController

object Drive {
    private val upAdjust: Double = 0.5
    private val downAdjust: Double = 0.25

    fun calculateSpeeds(controller: XboxController): ChassisSpeeds {
        val y = -controller.leftY
        val x = -controller.leftX
        val rot = -controller.rightY

        val triggerAdjust =
            (1 - upAdjust) + (controller.rightTriggerAxis * upAdjust) -
                (controller.leftTriggerAxis * downAdjust)

        return ChassisSpeeds(
            Units.MetersPerSecond.of(x * triggerAdjust),
            Units.MetersPerSecond.of(y * triggerAdjust),
            Units.RadiansPerSecond.of(rot * triggerAdjust),
        )
    }
}
