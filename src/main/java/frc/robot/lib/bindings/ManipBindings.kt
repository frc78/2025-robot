package frc.robot.lib.bindings

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.lib.Branch
import frc.robot.lib.Level
import frc.robot.lib.ScoreSelector
import frc.robot.lib.ScoreSelector.SelectedBranch
import frc.robot.lib.andWait
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Pivot
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.drivetrain.Chassis
import kotlin.math.absoluteValue
import org.littletonrobotics.junction.Logger

private val MANIPULATOR_LAYOUT =
    ManipulatorLayout.BUTTONS.also { Logger.recordMetadata("manip_layout", it.name) }

enum class ManipulatorLayout {
    MANUAL,
    DPAD,
    BUTTONS,
    LEFT_STICK,
    BOTH_STICKS,
}

fun CommandXboxController.configureManipulatorBindings() {
    when (MANIPULATOR_LAYOUT) {
        ManipulatorLayout.MANUAL -> configureManipManualLayout()
        ManipulatorLayout.DPAD -> configureManipDpadLayout()
        ManipulatorLayout.BUTTONS -> configureManipButtonLayout()
        ManipulatorLayout.LEFT_STICK -> configureManipLeftStickLayout()
        ManipulatorLayout.BOTH_STICKS -> configureManipBothSticksLayout()
    }
}

// Use buttons to manually go to levels
private fun CommandXboxController.configureManipManualLayout() {
    y().onTrue(SuperStructure.goToScoreCoral(RobotState.L4))
        .onFalse(SuperStructure.smartGoTo(RobotState.Stow))
    x().onTrue(SuperStructure.goToScoreCoral(RobotState.L3))
        .onFalse(SuperStructure.smartGoTo(RobotState.Stow))
    b().onTrue(SuperStructure.goToScoreCoral(RobotState.L2))
        .onFalse(SuperStructure.smartGoTo(RobotState.Stow))
    a().onTrue(SuperStructure.goToScoreCoral(RobotState.L1))
        .onFalse(SuperStructure.smartGoTo(RobotState.Stow))

    leftBumper()
        .onTrue(SuperStructure.smartGoTo(RobotState.CoralStation))
        .onFalse(SuperStructure.smartGoTo(RobotState.Stow))
    rightBumper()
        .onTrue(SuperStructure.smartGoTo(RobotState.AlgaeNet))
        .onFalse(SuperStructure.smartGoTo(RobotState.Stow))
    leftTrigger()
        .onTrue(SuperStructure.smartGoTo(RobotState.Processor))
        .onFalse(SuperStructure.smartGoTo(RobotState.Stow))
    rightTrigger()
        .onTrue(SuperStructure.smartGoTo(RobotState.AlgaeGroundPickup))
        .onFalse(SuperStructure.smartGoTo(RobotState.Stow))
}

/** Use dpad to select branch and level */
private fun CommandXboxController.configureManipDpadLayout() {
    povUp().onTrue(Commands.runOnce({ ScoreSelector.levelUp() }))
    povDown().onTrue(Commands.runOnce({ ScoreSelector.levelDown() }))
    povLeft().onTrue(Commands.runOnce({ SelectedBranch = Branch.LEFT }))
    povRight().onTrue(Commands.runOnce({ SelectedBranch = Branch.RIGHT }))
}

/** Use buttons to select branch and level */
private fun CommandXboxController.configureManipButtonLayout() {

    // Coral Stuff
    a().onTrue(SuperStructure.goToScoreCoral(RobotState.L1))
    b().onTrue(Commands.runOnce({ ScoreSelector.SelectedLevel = Level.L2 }))
    x().onTrue(Commands.runOnce({ ScoreSelector.SelectedLevel = Level.L3 }))
    y().onTrue(Commands.runOnce({ ScoreSelector.SelectedLevel = Level.L4 }))

    start().onTrue(SuperStructure.smartGoTo(RobotState.ReadyToClimb))
    back().onTrue(SuperStructure.smartGoTo(RobotState.FullyClimbed).alongWith(Climber.extend))

    rightTrigger(0.55)
        .onTrue(
            Intake.outtakeCoral
                .withDeadline(
                    // Wait until 0.2 seconds have passed and the trigger is released
                    Commands.waitSeconds(0.2)
                        .alongWith(Commands.waitUntil { rightTriggerAxis < 0.55 })
                )
                .andThen(SuperStructure.smartGoTo(RobotState.NewCoralStation))
        )

    rightBumper()
        .onTrue(Intake.intakeCoralThenHold())
        .whileTrue(SuperStructure.reachToIntake())
        .onFalse(
            Elevator.goTo(RobotState.CoralStation).andWait { Elevator.atPosition }
                .andThen(Commands.parallel(
                    Pivot.goTo(RobotState.CoralStation),
                    Wrist.goTo(RobotState.CoralStation)
                ))
        )
//        .whileTrue(Intake.intakeCoral.withName("Intake coral from coral station"))
//        .onFalse(Intake.holdCoral)

    // Algae Stuff
    povUp()
        .onTrue(
            SuperStructure.smartGoTo(RobotState.HighAlgaeIntake)
                .alongWith(Intake.intakeAlgaeThenHold()) // holds priority until algae is detected
                .andThen(SuperStructure.retractWithAlgae())
        )
    povDown()
        .onTrue(
            SuperStructure.smartGoTo(RobotState.LowAlgaeIntake)
                .alongWith(Intake.intakeAlgaeThenHold()) // holds priority until algae is detected
                .andThen(SuperStructure.retractWithAlgae())
        )
    povRight().onTrue(SuperStructure.smartGoTo(RobotState.AlgaeNet))
    povLeft().onTrue(SuperStructure.smartGoTo(RobotState.Processor))

    leftBumper()
        .onTrue(
            SuperStructure.smartGoTo(RobotState.AlgaeGroundPickup)
                .alongWith(Intake.intakeAlgaeThenHold()) // holds priority until algae is detected
                .andThen(SuperStructure.smartGoTo(RobotState.AlgaeStorage))
        )

    leftTrigger(0.55)
        .onTrue(Intake.scoreAlgae.andThen(SuperStructure.smartGoTo(RobotState.NewCoralStation)))
}

private fun CommandXboxController.configureManipLeftStickLayout() {
    axisGreaterThan(XboxController.Axis.kLeftX.value, 0.5)
        .onTrue(Commands.runOnce({ SelectedBranch = Branch.LEFT }))

    axisLessThan(XboxController.Axis.kLeftX.value, -0.5)
        .onTrue(Commands.runOnce({ SelectedBranch = Branch.RIGHT }))

    axisGreaterThan(XboxController.Axis.kLeftY.value, 0.5)
        .onTrue(Commands.runOnce({ ScoreSelector.levelDown() }))

    axisLessThan(XboxController.Axis.kLeftY.value, -0.5)
        .onTrue(Commands.runOnce({ ScoreSelector.levelUp() }))
}

// Idea for a more intuitive branch selector, which doesn't require reading the driver station
private fun CommandXboxController.configureManipBothSticksLayout() {
    leftBumper()
        .onTrue(
            Commands.runOnce({
                SelectedBranch = Branch.LEFT
                selectWithStick()
            })
        )
    rightBumper()
        .onTrue(
            Commands.runOnce({
                SelectedBranch = Branch.RIGHT
                selectWithStick()
            })
        )
}

private fun CommandXboxController.selectWithStick() {
    val t = 0.5 // Threshold
    val level =
        when {
            (leftY > t) && (rightY > t) -> Level.L4
            (leftY > t) && (rightY.absoluteValue < t) -> Level.L3
            (leftY.absoluteValue < t) && (rightY.absoluteValue < t) -> Level.L2
            (leftY < -t) -> Level.L1
            else -> null
        }
    level?.let { ScoreSelector.SelectedLevel = it }
}

/** Used for setting up a test controller / joystick */
fun CommandJoystick.configureManipTestBindings() {
    trigger().whileTrue(Chassis.measureWheelRotations)

    // SuperStructure manual control
    button(5).whileTrue(Pivot.moveUp)
    button(3).whileTrue(Pivot.moveDown)
    button(6).whileTrue(Elevator.manualUp)
    button(4).whileTrue(Elevator.manualDown)
    button(12).whileTrue(Wrist.manualUp)
    button(11).whileTrue(Wrist.manualDown)

    // Intake
    button(10).whileTrue(Intake.manualIntake)
    button(9).whileTrue(Intake.manualOuttake)
}
