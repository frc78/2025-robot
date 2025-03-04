package frc.robot.lib

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.lib.ScoreSelector.SelectedBranch
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Pivot
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.SuperStructure.goToMoveElevatorAndPivotTogether
import frc.robot.subsystems.Wrist
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
    y().onTrue(Commands.runOnce({ goToMoveElevatorAndPivotTogether(RobotState.L4) }))
    x().onTrue(Commands.runOnce({ goToMoveElevatorAndPivotTogether(RobotState.L3) }))
    b().onTrue(Commands.runOnce({ goToMoveElevatorAndPivotTogether(RobotState.L2) }))
    a().onTrue(Commands.runOnce({ goToMoveElevatorAndPivotTogether(RobotState.L1) }))
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
    //    leftBumper().onTrue(Commands.runOnce({ SelectedBranch = Branch.LEFT }))
    //    rightBumper().onTrue(Commands.runOnce({ SelectedBranch = Branch.RIGHT }))
    //    y().onTrue(Commands.runOnce({ SelectedLevel = Level.L4 }))
    //    a().onTrue(Commands.runOnce({ SelectedLevel = Level.L3 }))
    //    x().onTrue(Commands.runOnce({ SelectedLevel = Level.L2 }))
    //    b().onTrue(Commands.runOnce({ SelectedLevel = Level.L1 }))

    // Coral Stuff
    a().onTrue(SuperStructure.goToScoreCoral(RobotState.L1))
    b().onTrue(SuperStructure.goToScoreCoral(RobotState.L2))
    x().onTrue(SuperStructure.goToScoreCoral(RobotState.L3))
    y().onTrue(SuperStructure.goToScoreCoral(RobotState.L4)) // TODO test all these!

    //    a().onTrue(SuperStructure.smartGoTo(RobotState.L1))
    //    b().onTrue(SuperStructure.smartGoTo(RobotState.L2))
    //    x().onTrue(SuperStructure.smartGoTo(RobotState.L3))
    //    y().onTrue(SuperStructure.smartGoTo(RobotState.L4))

    rightTrigger(0.55)
        .onTrue(Intake.scoreCoral.andThen(SuperStructure.smartGoTo(RobotState.CoralStation)))
    // trigger value goes from 0 (not pressed) to 1 (fully pressed)
    rightBumper()
        .onTrue(
            SuperStructure.smartGoTo(RobotState.CoralStation)
                .alongWith(Intake.intakeCoralThenHold())
            //                .andThen(Commands.waitTime(1.seconds)) // TODO use robot pose here
            // instead of timer
            //                .andThen(SuperStructure.smartGoTo(RobotState.PreScore))
        )

    // Algae Stuff
    povUp()
        .onTrue(
            SuperStructure.smartGoTo(RobotState.HighAlgaeIntake)
                .alongWith(Intake.intakeAlgaeThenHold()) // holds priority until algae is detected
                .andThen(SuperStructure.smartGoTo(RobotState.PreScore))
        )
    povDown()
        .onTrue(
            SuperStructure.smartGoTo(RobotState.LowAlgaeIntake)
                .alongWith(Intake.intakeAlgaeThenHold()) // holds priority until algae is detected
                .andThen(SuperStructure.smartGoTo(RobotState.PreScore))
        )
    povRight().onTrue(SuperStructure.smartGoTo(RobotState.AlgaeNet))
    povLeft().onTrue(SuperStructure.smartGoTo(RobotState.Processor))

    leftBumper()
        .onTrue(
            SuperStructure.smartGoTo(RobotState.AlgaeGroundPickup)
                .alongWith(Intake.intakeAlgaeThenHold()) // holds priority until algae is detected
                .andThen(SuperStructure.smartGoTo(RobotState.PreScore))
        )

    leftTrigger(0.55)
        .onTrue(Intake.scoreAlgae.andThen(SuperStructure.smartGoTo(RobotState.CoralStation)))
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
    button(5).whileTrue(Pivot.moveUp)
    button(3).whileTrue(Pivot.moveDown)
    button(6).whileTrue(Elevator.manualUp)
    button(4).whileTrue(Elevator.manualDown)

    // Pivot control tuning
    //    button(7).onTrue(Pivot.goToRawUntil(10.degrees){true})
    //    button(8).onTrue(Pivot.goToRawUntil(66.degrees){true}) // coral station angle
    //    button(9).onTrue(Pivot.goToRawUntil(90.degrees){true})

    // Elevator control tuning
    //    button(10).onTrue(Elevator.goToRawUntil(0.25.inches){true})
    //    button(11).onTrue(Elevator.goToRawUntil(20.inches){true}) // l3 height
    //    button(12).onTrue(Elevator.goToRawUntil(50.inches){true}) // l4 height

    // Wrist control tuning
    //    button(10).onTrue(Wrist.goToRawUntil(13.degrees){true})
    //    button(11).onTrue(Wrist.goToRawUntil(120.degrees){true}) // l3 height
    //    button(12).onTrue(Wrist.goToRawUntil(165.degrees){true}) // l4 height

    button(7).onTrue(SuperStructure.smartGoTo(RobotState.ReadyToClimb))
    button(8)
        .onTrue(
            Wrist.goTo(RobotState.FullyClimbed)
                .alongWith(Elevator.goTo(RobotState.FullyClimbed))
                .alongWith(
                    Pivot.goToRawUntil(RobotState.FullyClimbed.pivotAngle) {
                        Pivot.angle < Pivot.EXTEND_FOOT_THRESHOLD
                    }
                )
                .andThen(Climber.extend)
        )

    button(9).whileTrue(Climber.extend)
    button(10).whileTrue(Climber.retract)
    button(11).whileTrue(Wrist.manualUp())
    button(12).whileTrue(Wrist.manualDown())
}
