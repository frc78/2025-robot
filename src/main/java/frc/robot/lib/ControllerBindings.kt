package frc.robot.lib

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.lib.ScoreSelector.SelectedBranch
import frc.robot.lib.ScoreSelector.SelectedLevel
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Pivot
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.SuperStructure.goTo
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.drivetrain.Chassis

private val MANIPULATOR_LAYOUT =
    ManipulatorLayout.LEFT_STICK.also { SmartDashboard.putString("manip_layout", it.name) }

fun CommandXboxController.configureDriverBindings() {
    rightBumper()
        .whileTrue(
            Chassis.driveToSelectedBranch
                .andThen(SuperStructure.goToSelectedLevel)
                .andThen(Intake.scoreCoral)
                .andThen(goTo(RobotState.Stow))
        )

    leftBumper().whileTrue(Chassis.driveToLeftBranch)
    rightBumper().whileTrue(Chassis.driveToRightBranch)
    y().whileTrue(Elevator.manualUp)
    a().whileTrue(Elevator.manualDown)
    start().onTrue(Commands.runOnce({ Chassis.zeroHeading }))

    Chassis.defaultCommand =
        Chassis.fieldCentricDrive {
            withVelocityX(hid.velocityX)
                .withVelocityY(hid.velocityY)
                .withRotationalRate(hid.velocityRot)
        }

    x().whileTrue(Chassis.snapToReef { withVelocityX(hid.velocityX).withVelocityY(hid.velocityY) })
}

enum class ManipulatorLayout {
    MANUAL,
    DPAD,
    BUTTONS,
    LEFT_STICK,
}

fun CommandXboxController.configureManipulatorBindings() {
    when (MANIPULATOR_LAYOUT) {
        ManipulatorLayout.MANUAL -> configureManualLayout()
        ManipulatorLayout.DPAD -> configureDpadLayout()
        ManipulatorLayout.BUTTONS -> configureButtonLayout()
        ManipulatorLayout.LEFT_STICK -> configureLeftStickLayout()
    }
}

// Use buttons to manually go to levels
private fun CommandXboxController.configureManualLayout() {
    y().onTrue(Commands.runOnce({ goTo(RobotState.L4) }))
    x().onTrue(Commands.runOnce({ goTo(RobotState.L3) }))
    b().onTrue(Commands.runOnce({ goTo(RobotState.L2) }))
    a().onTrue(Commands.runOnce({ goTo(RobotState.L1) }))
}

/** Use dpad to select branch and level */
private fun CommandXboxController.configureDpadLayout() {
    povUp().onTrue(Commands.runOnce({ ScoreSelector.levelUp() }))
    povDown().onTrue(Commands.runOnce({ ScoreSelector.levelDown() }))
    povLeft().onTrue(Commands.runOnce({ SelectedBranch = Branch.LEFT }))
    povRight().onTrue(Commands.runOnce({ SelectedBranch = Branch.RIGHT }))
}

/** Use buttons to select branch and level */
private fun CommandXboxController.configureButtonLayout() {
    leftBumper().onTrue(Commands.runOnce({ SelectedBranch = Branch.LEFT }))
    rightBumper().onTrue(Commands.runOnce({ SelectedBranch = Branch.RIGHT }))
    y().onTrue(Commands.runOnce({ SelectedLevel = Level.L4 }))
    a().onTrue(Commands.runOnce({ SelectedLevel = Level.L3 }))
    x().onTrue(Commands.runOnce({ SelectedLevel = Level.L2 }))
    b().onTrue(Commands.runOnce({ SelectedLevel = Level.L1 }))
}

private fun CommandXboxController.configureLeftStickLayout() {
    axisGreaterThan(XboxController.Axis.kLeftX.value, 0.5)
        .onTrue(Commands.runOnce({ SelectedBranch = Branch.LEFT }))

    axisLessThan(XboxController.Axis.kLeftX.value, -0.5)
        .onTrue(Commands.runOnce({ SelectedBranch = Branch.RIGHT }))

    axisGreaterThan(XboxController.Axis.kLeftY.value, 0.5)
        .onTrue(Commands.runOnce({ ScoreSelector.levelDown() }))

    axisLessThan(XboxController.Axis.kLeftY.value, -0.5)
        .onTrue(Commands.runOnce({ ScoreSelector.levelUp() }))
}

/** Used for setting up a test controller / joystick */
fun CommandJoystick.configureTestBindings() {
    button(5).whileTrue(Pivot.moveUp)
    button(3).whileTrue(Pivot.moveDown)
    button(6).whileTrue(Elevator.manualUp)
    button(4).whileTrue(Elevator.manualDown)
    button(7).whileTrue(Intake.intakeCoralThenHold())
    button(8).whileTrue(Intake.outtakeCoral)
    button(9).whileTrue(Intake.intakeAlgae)
    button(10).whileTrue(Intake.outtakeAlgae)
    button(11).whileTrue(Wrist.manualUp())
    button(12).whileTrue(Wrist.manualDown())
    trigger().whileTrue(Climber.climb())
}
