package frc.robot.lib.bindings

import edu.wpi.first.wpilibj2.command.button.CommandXboxController

object ReefscapeController {
    val controller = CommandXboxController(0)

    fun home() = controller.povLeft()

    fun l1() = controller.a()

    fun l2() = controller.b()

    fun lowAlgae() = controller.b()

    fun l3() = controller.x()

    fun highAlgae() = controller.x()

    fun l4() = controller.y()

    fun net() = controller.y()

    fun floorAlgae() = controller.povDown()

    fun process() = controller.povDown()

    fun score() = controller.rightBumper()

    fun coral() = controller.leftBumper()

    fun prepareClimb() = controller.start()

    fun climb() = controller.back()
}
