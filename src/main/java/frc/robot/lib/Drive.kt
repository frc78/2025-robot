package frc.robot.lib

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.XboxController

private const val UP_ADJUST = 0.5
private const val DOWN_ADJUST = 0.4
private const val JOYSTICK_DEADBAND = 0.1
private const val TRIGGER_DEADBAND = 0.05
private val maxTranslation = 3.metersPerSecond
private val maxRotation = 1.rotationsPerSecond

// Extension properties to access the drive velocities based on joystick input
// This used to be a single method that returned a ChassisSpeeds object,
// but it made it hard to use in cases where we wanted an override of one or more speeds,
// such as auto-alignment

// triggerAdjust is a scalar that allows scaling up or down based on the values of the left and
// right triggers. The output is between (1-UP_ADJUST-DOWN_ADJUST) and 1 (default 0.1 and 1)
val XboxController.triggerAdjust
    get() =
        (1 - UP_ADJUST) + (MathUtil.applyDeadband(rightTriggerAxis, TRIGGER_DEADBAND) * UP_ADJUST) -
            (MathUtil.applyDeadband(leftTriggerAxis, TRIGGER_DEADBAND) * DOWN_ADJUST)

val obstacleSlowdown
    get() = null

val XboxController.velocityX: LinearVelocity
    get() {
        val x = MathUtil.applyDeadband(-leftY, JOYSTICK_DEADBAND)
        return maxTranslation * x * triggerAdjust
    }

val XboxController.velocityY: LinearVelocity
    get() {
        val y = MathUtil.applyDeadband(-leftX, JOYSTICK_DEADBAND)
        return maxTranslation * y * triggerAdjust
    }

val XboxController.velocityRot: AngularVelocity
    get() {
        val rot = MathUtil.applyDeadband(-rightX, JOYSTICK_DEADBAND)
        return maxRotation * rot * triggerAdjust
    }
