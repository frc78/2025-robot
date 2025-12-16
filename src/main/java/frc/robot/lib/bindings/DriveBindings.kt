package frc.robot.lib.bindings

import com.pathplanner.lib.events.EventTrigger
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.scoreCoralWhenClose
import frc.robot.lib.command
import frc.robot.lib.velocityRot
import frc.robot.lib.velocityX
import frc.robot.lib.velocityY
import frc.robot.subsystems.Intake
import frc.robot.subsystems.LEDSubsystem
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.drivetrain.Chassis

val TRIGGER_ADJUST = true.also { SmartDashboard.putBoolean("trigger_adjust", it) }
val DISTANCE_SLOWING = true.also { SmartDashboard.putBoolean("distance_slowing", it) }

fun CommandXboxController.configureDriverBindings() {
    configureDriveBasicLayout()
    configureDriveAutomaticSequencingLayout()
}

// Basic driving
fun CommandXboxController.configureDriveBasicLayout() {
    Chassis.defaultCommand =
        Chassis.fieldCentricDrive {
                withVelocityX(hid.velocityX)
                    .withVelocityY(hid.velocityY)
                    .withRotationalRate(hid.velocityRot)
            }
            .withName("Field centric xbox drive")
    // Rumble for short duration on game piece acquisition
    Trigger { Intake.holdingCoral || Intake.holdingAlgae }
        .onTrue(
            Commands.startEnd(
                    { setRumble(GenericHID.RumbleType.kBothRumble, 1.0) },
                    { setRumble(GenericHID.RumbleType.kBothRumble, 0.0) },
                )
                .withTimeout(0.5)
        )
}

// Driving with buttons for automatic scoring sequences
private fun CommandXboxController.configureDriveAutomaticSequencingLayout() {
    val notLeftBumper = leftBumper().negate()
    val notRightBumper = rightBumper().negate()

    val atApproachPoint = EventTrigger("atApproachPoint")
    // Auto score algae in processor
    b().whileTrue(
            Chassis.driveToProcessor
                .andThen(Intake.scoreAlgae)
                .andThen(Chassis.backAwayFromProcessor.until { Chassis.isWithinGoal(0.05) })
                .andThen(SuperStructure.smartGoTo(RobotState.Stow).asProxy())
        )
        // Once at approach point, move to processor position
        .and(atApproachPoint)
        .onTrue(SuperStructure.smartGoTo(RobotState.Processor))

    a().and(notRightBumper)
        .and(notLeftBumper)
        .whileTrue(Chassis.driveToClosestCenterCoralStation.raceWith(LEDSubsystem.flashWhite))
    a().onTrue(SuperStructure.smartGoTo(RobotState.NewCoralStation).alongWith(Intake.intakeCoral))

    // Adding this to the automatic sequencing layout per Eli's request at Battlecry
    x().whileTrue(
        Chassis.snapAngleToReef { withVelocityX(hid.velocityX).withVelocityY(hid.velocityY) }
    )

    configureReefAlignments()
    configureBargeAlignments()
}

private fun CommandXboxController.configureReefAlignments() {
    val notLeftBumper = leftBumper().negate()
    val notRightBumper = rightBumper().negate()
    val hasCoral = Trigger { Intake.holdingCoral }

    rightBumper()
        .and(notLeftBumper)
        .and(hasCoral)
        .whileTrue(
            Chassis.driveToRightBranch
                .alongWith(scoreCoralWhenClose)
                .raceWith(LEDSubsystem.flashForSelectedLevel)
        )
        .onFalse(
            ConditionalCommand(
                SuperStructure.smartGoTo(RobotState.CoralStorage),
                SuperStructure.smartGoTo(RobotState.NewCoralStation), // todo smoother retraction?
            ) {
                Intake.holdingCoral
            }
        )

    leftBumper()
        .and(notRightBumper)
        .and(hasCoral)
        .whileTrue(
            Chassis.driveToLeftBranch
                .alongWith(scoreCoralWhenClose)
                .raceWith(LEDSubsystem.flashForSelectedLevel)
        )
        .onFalse(
            ConditionalCommand(
                SuperStructure.smartGoTo(RobotState.CoralStorage),
                SuperStructure.smartGoTo(RobotState.NewCoralStation), // todo smoother retraction?
            ) {
                Intake.holdingCoral
            }
        )

    leftBumper()
        .and(rightBumper())
        .and(hasCoral.negate())
        .whileTrue(
            Chassis.driveToClosestReef
                .alongWith(
                    Commands.sequence(
                        Commands.waitUntil { Chassis.isWithinGoal(1.5) },
                        SuperStructure.retrieveAlgaeFromReef,
                    )
                )
                .raceWith(LEDSubsystem.flashForSelectedLevel)
        )
        .onFalse(SuperStructure.retractWithAlgae())
}

private fun CommandXboxController.configureBargeAlignments() {
    val notLeftBumper = leftBumper().negate()
    val notRightBumper = rightBumper().negate()
    val atApproachPoint = EventTrigger("atApproachPoint")

    val scoreAlgaeSequence by command {
        SuperStructure.smartGoTo(RobotState.AlgaeNet)
            .alongWith(
                Commands.waitUntil { Chassis.isWithinGoal(0.06) && SuperStructure.atPosition }
                    .andThen(Intake.scoreAlgae)
            )
            .andThen(SuperStructure.smartGoTo(RobotState.NewCoralStation))
            .onlyIf { Intake.holdingAlgae }
    }
    // only y
    y().and(notLeftBumper)
        .and(notRightBumper)
        .whileTrue(Chassis.driveToBarge.raceWith(LEDSubsystem.flashPink))
        .and(atApproachPoint)
        .onTrue(scoreAlgaeSequence)
    // y and left bumper
    y().and(leftBumper())
        .and(notRightBumper)
        .whileTrue(Chassis.driveToBargeLeft.raceWith(LEDSubsystem.flashPink))
        .and(atApproachPoint)
        .onTrue(scoreAlgaeSequence)
    // y and right bumper
    y().and(rightBumper())
        .and(notLeftBumper)
        .whileTrue(Chassis.driveToBargeRight.raceWith(LEDSubsystem.flashPink))
        .and(atApproachPoint)
        .onTrue(scoreAlgaeSequence)

    // y released, retract to algae storage with algae or CoralStation without
    y().onFalse(
        ConditionalCommand(
            SuperStructure.smartGoTo(RobotState.AlgaeStorage),
            SuperStructure.smartGoTo(
                RobotState.NewCoralStation
            ), // spencer changed from stow thurs afternoon to avoid catching on barge
        ) {
            Intake.holdingAlgae
        }
    )
}
