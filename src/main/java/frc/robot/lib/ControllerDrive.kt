package frc.robot.lib

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.XboxController
import kotlin.math.abs

private const val JOYSTICK_DEADBAND = 0.1
private val maxTranslation = 4.0.metersPerSecond
private val maxRotation = 1.5.rotationsPerSecond

/* Extension properties to access the drive velocities based on joystick input
 * This used to be a single method that returned a ChassisSpeeds object,
 * but it made it hard to use in cases where we wanted an override of one or more speeds,
 * such as auto-alignment */

val XboxController.velocityX: LinearVelocity
    get() {
        val x = MathUtil.applyDeadband(-leftY, JOYSTICK_DEADBAND)
        // Square inputs for finer control at low speeds
        return maxTranslation * x * abs(x)
    }

val XboxController.velocityY: LinearVelocity
    get() {
        val y = MathUtil.applyDeadband(-leftX, JOYSTICK_DEADBAND)
        // Square inputs for finer control at low speeds
        return maxTranslation * y * abs(y)
    }

val XboxController.velocityRot: AngularVelocity
    get() {
        val rot = MathUtil.applyDeadband(-rightX, JOYSTICK_DEADBAND)
        // Square inputs for finer control at low speeds
        return maxRotation * rot * abs(rot)
    }
