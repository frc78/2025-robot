package frc.robot.lib

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.XboxController
import frc.robot.lib.bindings.DISTANCE_SLOWING
import frc.robot.lib.bindings.TRIGGER_ADJUST
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.drivetrain.Chassis
import org.littletonrobotics.junction.Logger

private const val UP_ADJUST = 0.25
private const val DOWN_ADJUST = 0.55 // changed from 0.4 at 1821 3/8
private const val JOYSTICK_DEADBAND = 0.1
private const val TRIGGER_DEADBAND = 0.05
private val maxTranslation = 4.0.metersPerSecond
private val maxRotation = 1.5.rotationsPerSecond

/* Extension properties to access the drive velocities based on joystick input
 * This used to be a single method that returned a ChassisSpeeds object,
 * but it made it hard to use in cases where we wanted an override of one or more speeds,
 * such as auto-alignment */

/**
 * Calculates a speed modifier that allows scaling up or down based on the values of the left and
 * right triggers. The output is between (1-UP_ADJUST-DOWN_ADJUST) and 1
 */
val XboxController.triggerAdjust
    get() =
        (1 - UP_ADJUST) + (MathUtil.applyDeadband(rightTriggerAxis, TRIGGER_DEADBAND) * UP_ADJUST) -
            (MathUtil.applyDeadband(leftTriggerAxis, TRIGGER_DEADBAND) * DOWN_ADJUST)

val obstacleSlowdown
    get() =
        distanceSlowdown(
                FieldGeometry.distanceToClosestLine(
                        FieldGeometry.CORAL_STATIONS,
                        Chassis.state.Pose.translation,
                    )
                    .also { Logger.recordOutput("closestCoralStationDistance", it) },
                startSlowDistance = 1.0,
                maxSlowDistance = 0.75,
                maxSlowdownCoefficient = 0.4,
            )
            .also { Logger.recordOutput("obstacleSlowdown", it) }

/** Cumulates the enabled speed modifiers into one coefficient */
val XboxController.speedModifiers
    get() =
        // TODO refactor this to be at all readable, or at least some comments
        (if (TRIGGER_ADJUST) triggerAdjust else 1.0) *
            (if (DISTANCE_SLOWING && if (RobotBase.isReal()) !Intake.hasBranchCoral else false)
                obstacleSlowdown
            else 1.0) *
            // scale speed down with elevator height, full speed under 1 inch extension down to .15
            // multiplier at max height
            (if (Elevator.position > 1.inches)
                ((Elevator.MAX_HEIGHT -
                        RobotState.L4.elevatorHeight
                            .baseUnitMagnitude()
                            .coerceAtMost(Elevator.position.baseUnitMagnitude())
                            .inches) / Elevator.MAX_HEIGHT)
                    .baseUnitMagnitude() * 0.3
            else 1.0)

val XboxController.velocityX: LinearVelocity
    get() {
        val x = MathUtil.applyDeadband(-leftY, JOYSTICK_DEADBAND)
        return maxTranslation * x * speedModifiers
    }

val XboxController.wideVelocityX: LinearVelocity
    get() {
        val x = MathUtil.applyDeadband(-leftY, 0.5)
        return maxTranslation * x * speedModifiers
    }

val XboxController.velocityY: LinearVelocity
    get() {
        val y = MathUtil.applyDeadband(-leftX, JOYSTICK_DEADBAND)
        return maxTranslation * y * speedModifiers
    }

val XboxController.velocityRot: AngularVelocity
    get() {
        val rot = MathUtil.applyDeadband(-rightX, JOYSTICK_DEADBAND)
        return maxRotation * rot * speedModifiers
    }

/**
 * Calculates a speed modifier based on a distance and a range. Note: 0 < maxSlowDistance <
 * startSlowDistance
 *
 * @param distance The measured distance
 * @param startSlowDistance The distance at which the robot will start slowing down
 * @param maxSlowDistance The distance at which the effect will be greatest, i.e. slowdown factor =
 *   maxSlowdownCoefficient
 * @param maxSlowdownCoefficient The maximum decrease in speed this modifier will apply (0.1 will
 *   decrease speed by 10% FROM original speed)
 * @return Value between 1.0 and (1.0 - maxSlowdownCoefficient)
 */
private fun distanceSlowdown(
    distance: Double,
    startSlowDistance: Double, // Distance at which the robot starts slowing down
    maxSlowDistance: Double, // Distance by which slowing down factor is maximum
    maxSlowdownCoefficient: Double, // Maximum slowdown factor
): Double {
    val maxValue = startSlowDistance - maxSlowDistance
    val inverseInterpolate = MathUtil.clamp(startSlowDistance - distance, 0.0, maxValue) / maxValue
    Logger.recordOutput("distanceClamp", inverseInterpolate)
    return 1 - (inverseInterpolate * (1 - maxSlowdownCoefficient))
}
