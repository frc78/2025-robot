package frc.robot.lib.bindings

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.XboxController
import frc.robot.lib.metersPerSecond
import frc.robot.lib.rotationsPerSecond
import kotlin.math.abs

object ReefscapeController {
    private const val JOYSTICK_DEADBAND = 0.1
    private val maxTranslation = 4.0.metersPerSecond
    private val maxRotation = 1.5.rotationsPerSecond

    private const val POV_UP = 0
    private const val POV_RIGHT = 90
    private const val POV_DOWN = 180
    private const val POV_LEFT = 270

    val controller = XboxController(0)

    // POV Up
    fun home() = controller.aButton

    fun l1() = controller.pov == POV_UP

    fun l2() = controller.bButton

    fun lowAlgae() = controller.bButton

    fun l3() = controller.xButton

    fun highAlgae() = controller.xButton

    fun l4() = controller.yButton

    fun net() = controller.yButton

    fun floorAlgae() = controller.pov == POV_DOWN

    fun process() = controller.pov == POV_DOWN

    fun score() = controller.rightBumperButton

    fun coral() = controller.leftBumperButton

    fun prepareClimb() = controller.startButton

    fun climb() = controller.backButton

    fun robotCentric() = controller.rightStickButton

    fun fieldCentric() = controller.leftStickButton

    val velocityX: LinearVelocity
        get() {
            val x = MathUtil.applyDeadband(-controller.leftY, JOYSTICK_DEADBAND)
            // Square inputs for finer control at low speeds
            return maxTranslation * x * abs(x)
        }

    val velocityY: LinearVelocity
        get() {
            val y = MathUtil.applyDeadband(-controller.leftX, JOYSTICK_DEADBAND)
            // Square inputs for finer control at low speeds
            return maxTranslation * y * abs(y)
        }

    val velocityRot: AngularVelocity
        get() {
            val rot = MathUtil.applyDeadband(-controller.rightX, JOYSTICK_DEADBAND)
            // Square inputs for finer control at low speeds
            return maxRotation * rot * abs(rot)
        }
}
