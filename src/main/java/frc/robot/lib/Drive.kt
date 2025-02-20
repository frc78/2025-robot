package frc.robot.lib

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.XboxController

private const val UP_ADJUST = 0.5
private const val DOWN_ADJUST = 0.25
private val maxSpeed = 3.metersPerSecond
private val maxRotSpeed = 1.rotationsPerSecond

fun XboxController.calculateSpeeds(): ChassisSpeeds {
    val y = -MathUtil.applyDeadband(leftX, 0.1)
    val x = -MathUtil.applyDeadband(leftY, 0.1)
    val rot = -MathUtil.applyDeadband(rightX, 0.1)
    val triggerAdjust =
        (1 - UP_ADJUST) + (rightTriggerAxis * UP_ADJUST) - (leftTriggerAxis * DOWN_ADJUST)
    return ChassisSpeeds(
        maxSpeed.metersPerSecond * x * triggerAdjust,
        maxSpeed.metersPerSecond * y * triggerAdjust,
        maxRotSpeed.radiansPerSecond * rot * triggerAdjust,
    )
}
