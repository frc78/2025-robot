package frc.robot.lib.bindings

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.lib.Level
import frc.robot.lib.ScoreSelector
import frc.robot.lib.andWait
import frc.robot.subsystems.*

fun CommandXboxController.configureManipulatorBindings() {

    // Coral Stuff
    a().onTrue(SuperStructure.goToScoreCoral(RobotState.L1))
    b().onTrue(Commands.runOnce({ ScoreSelector.SelectedLevel = Level.L2 }))
    x().onTrue(Commands.runOnce({ ScoreSelector.SelectedLevel = Level.L3 }))
    y().onTrue(Commands.runOnce({ ScoreSelector.SelectedLevel = Level.L4 }))

    start().onTrue(SuperStructure.smartGoTo(RobotState.ReadyToClimb))
    back().onTrue(SuperStructure.smartGoTo(RobotState.FullyClimbed).alongWith(Climber.extend))

    rightBumper().onTrue(Intake.intakeCoral)

    configManipButtonLayoutAlgae()
}

private fun CommandXboxController.configManipButtonLayoutAlgae() {
    // Manual pivot adjustment in case intake alignment is bad
    povUp().whileTrue(Pivot.moveUp)
    povDown().whileTrue(Pivot.moveDown)
    povRight().onTrue(SuperStructure.smartGoTo(RobotState.AlgaeNet))
    povLeft().onTrue(SuperStructure.smartGoTo(RobotState.Processor))
    leftBumper()
        .onTrue(
            SuperStructure.smartGoTo(RobotState.AlgaeGroundPickup)
                .alongWith(
                    Intake.intakeAlgae.andWait { Intake.holdingAlgae }
                ) // holds priority until algae is detected
                .andThen(SuperStructure.smartGoTo(RobotState.AlgaeStorage))
        )
    leftTrigger(0.55)
        .onTrue(Intake.scoreAlgae.andThen(SuperStructure.smartGoTo(RobotState.NewCoralStation)))
}
