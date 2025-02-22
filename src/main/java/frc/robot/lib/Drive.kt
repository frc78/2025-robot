package frc.robot.lib

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.XboxController

private const val UP_ADJUST = 0.5
private const val DOWN_ADJUST = 0.25
private const val DEADBAND = 0.1
private val maxTranslation = 3.metersPerSecond
private val maxRotation = 1.rotationsPerSecond

// These use raw doubles to avoid allocations every loop
val XboxController.velocityX: LinearVelocity
    get() {
        val x = MathUtil.applyDeadband(-leftY, DEADBAND)
        val triggerAdjust =
            (1 - UP_ADJUST) + (rightTriggerAxis * UP_ADJUST) - (leftTriggerAxis * DOWN_ADJUST)
        return maxTranslation * x * triggerAdjust
    }

val XboxController.velocityY: LinearVelocity
    get() {
        val y = MathUtil.applyDeadband(-leftX, DEADBAND)
        val triggerAdjust =
            (1 - UP_ADJUST) + (rightTriggerAxis * UP_ADJUST) - (leftTriggerAxis * DOWN_ADJUST)
        return maxTranslation * y * triggerAdjust
    }

val XboxController.velocityRot: AngularVelocity
    get() {
        val rot = MathUtil.applyDeadband(-rightX, DEADBAND)
        val triggerAdjust =
            (1 - UP_ADJUST) + (rightTriggerAxis * UP_ADJUST) - (leftTriggerAxis * DOWN_ADJUST)
        return maxRotation * rot * triggerAdjust
    }
