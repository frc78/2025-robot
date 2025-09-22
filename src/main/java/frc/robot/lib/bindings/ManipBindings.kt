package frc.robot.lib.bindings

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.lib.Level
import frc.robot.lib.ScoreSelector
import frc.robot.lib.andWait
import frc.robot.lib.inches
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Pivot
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.drivetrain.Chassis

@Suppress("LongMethod")
fun CommandXboxController.configureManipulatorBindings() {

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
        .onTrue(Intake.intakeCoralThenHold)
        .onFalse(
            Elevator.goTo(RobotState.NewCoralStation)
                .andWait { Elevator.position < 3.inches }
                .andThen(
                    Commands.parallel(
                        Pivot.goTo(RobotState.NewCoralStation),
                        Wrist.goTo(RobotState.NewCoralStation),
                    )
                )
        )
    // Algae Stuff
    povUp()
        .onTrue(
            SuperStructure.smartGoTo(RobotState.HighAlgaeIntake)
                .alongWith(Intake.intakeAlgaeThenHold) // holds priority until algae is detected
                .andThen(SuperStructure.retractWithAlgae())
        )
    povDown()
        .onTrue(
            SuperStructure.smartGoTo(RobotState.LowAlgaeIntake)
                .alongWith(Intake.intakeAlgaeThenHold) // holds priority until algae is detected
                .andThen(SuperStructure.retractWithAlgae())
        )
    povRight().onTrue(SuperStructure.smartGoTo(RobotState.AlgaeNet))
    povLeft().onTrue(SuperStructure.smartGoTo(RobotState.Processor))
    leftBumper()
        .onTrue(
            SuperStructure.smartGoTo(RobotState.AlgaeGroundPickup)
                .alongWith(Intake.intakeAlgaeThenHold) // holds priority until algae is detected
                .andThen(SuperStructure.smartGoTo(RobotState.AlgaeStorage))
        )
    leftTrigger(0.55)
        .onTrue(Intake.scoreAlgae.andThen(SuperStructure.smartGoTo(RobotState.NewCoralStation)))
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
