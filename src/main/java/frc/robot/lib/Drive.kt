package frc.robot.lib

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.XboxController

private const val UP_ADJUST = 0.5
private const val DOWN_ADJUST = 0.25
private val maxSpeed = 3.metersPerSecond
private val maxRotSpeed = 1.rotationsPerSecond

fun XboxController.calculateSpeeds(): ChassisSpeeds {
    val y = -leftX
    val x = -leftY
    val rot = -rightX
    val triggerAdjust =
        (1 - UP_ADJUST) + (rightTriggerAxis * UP_ADJUST) - (leftTriggerAxis * DOWN_ADJUST)
    return ChassisSpeeds(
        maxSpeed.metersPerSecond * x * triggerAdjust,
        maxSpeed.metersPerSecond * y * triggerAdjust,
        maxRotSpeed.radiansPerSecond * rot * triggerAdjust,
    )
}
