package frc.robot.lib

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.XboxController

private val upAdjust: Double = 0.5
private val downAdjust: Double = 0.25
private val maxSpeed: LinearVelocity = Units.MetersPerSecond.of(3.0)
private val maxRotSpeed: AngularVelocity = Units.RadiansPerSecond.of(2.0 * Math.PI)

fun XboxController.calculateSpeeds(): ChassisSpeeds {
    val y = -leftY
    val x = -leftX
    val rot = -rightX
    val triggerAdjust =
        (1 - upAdjust) + (rightTriggerAxis * upAdjust) - (leftTriggerAxis * downAdjust)
    return ChassisSpeeds(
        Units.MetersPerSecond.of(x * triggerAdjust * maxSpeed.`in`(Units.MetersPerSecond)),
        Units.MetersPerSecond.of(y * triggerAdjust * maxSpeed.`in`(Units.MetersPerSecond)),
        Units.RadiansPerSecond.of(rot * triggerAdjust * maxRotSpeed.`in`(Units.RadiansPerSecond)),
    )
}
