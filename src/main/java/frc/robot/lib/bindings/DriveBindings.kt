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

private val DRIVE_LAYOUT =
    DriveLayout.AUTOMATIC_SEQUENCING.also { SmartDashboard.putString("drive_layout", it.name) }

val TRIGGER_ADJUST = true.also { SmartDashboard.putBoolean("trigger_adjust", it) }
val DISTANCE_SLOWING = true.also { SmartDashboard.putBoolean("distance_slowing", it) }

private enum class DriveLayout {
    BASIC,
    SNAPPING,
    MANUAL_SEQUENCING,
    AUTOMATIC_SEQUENCING,
    ALIGN_TESTING,
}

fun CommandXboxController.configureDriverBindings() {
    configureDriveBasicLayout()

    when (DRIVE_LAYOUT) {
        DriveLayout.BASIC -> {
            /*Already configured by default*/
        }
        DriveLayout.SNAPPING -> configureDriveSnappingLayout()
        DriveLayout.MANUAL_SEQUENCING -> configureDriveManualSequencingLayout()
        DriveLayout.AUTOMATIC_SEQUENCING -> configureDriveAutomaticSequencingLayout()
        DriveLayout.ALIGN_TESTING -> configureAutoAlignTestingLayout()
    }
}

fun CommandXboxController.configureAutoAlignTestingLayout() {

    val notLeftBumper = leftBumper().negate()
    val notRightBumper = rightBumper().negate()

    // only y
    y().and(notLeftBumper).and(notRightBumper).whileTrue(Chassis.driveToBarge)
    // y and left bumper
    y().and(leftBumper()).and(notRightBumper).whileTrue(Chassis.driveToBargeLeft)
    // y and right bumper
    y().and(rightBumper()).and(notLeftBumper).whileTrue(Chassis.driveToBargeRight)
    val hasCoral = Trigger { Intake.hasBranchCoral }
    val hasNoCoral = Trigger { !Intake.hasBranchCoral }

    rightBumper().and(notLeftBumper).whileTrue(Chassis.driveToRightBranch)

    leftBumper().and(notRightBumper).whileTrue(Chassis.driveToLeftBranch)

    leftBumper().and(rightBumper()).whileTrue(Chassis.driveToClosestReef)
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
    Trigger { Intake.hasCoralByCurrent() }
        .onTrue(
            Commands.startEnd(
                    { setRumble(GenericHID.RumbleType.kBothRumble, 1.0) },
                    { setRumble(GenericHID.RumbleType.kBothRumble, 0.0) },
                )
                .withTimeout(0.5)
        )

    Trigger { Intake.detectAlgaeByCurrent() }
        .onTrue(
            Commands.startEnd(
                    { setRumble(GenericHID.RumbleType.kBothRumble, 1.0) },
                    { setRumble(GenericHID.RumbleType.kBothRumble, 0.0) },
                )
                .withTimeout(0.5)
        )
}

// Driving with snapping bindings for reef alignment
private fun CommandXboxController.configureDriveSnappingLayout() {
    val notLeftBumper = leftBumper().negate()
    val notRightBumper = rightBumper().negate()
    val notA = a().negate()
    val notY = y().negate()
    // only left bumper
    leftBumper().and(notRightBumper).and(notA).and(notY).whileTrue(Chassis.driveToLeftBranch)
    // only right bumper
    rightBumper().and(notLeftBumper).and(notA).and(notY).whileTrue(Chassis.driveToRightBranch)

    // both left and right bumper
    leftBumper().and(rightBumper()).and(notA).and(notY).whileTrue(Chassis.driveToClosestReef)

    a().whileTrue(
        Chassis.snapAngleToCoralStation {
            withVelocityX(hid.velocityX).withVelocityY(hid.velocityY)
        }
    )

    x().whileTrue(
        Chassis.snapAngleToReef { withVelocityX(hid.velocityX).withVelocityY(hid.velocityY) }
    )

    // only y
    y().and(notLeftBumper).and(notRightBumper).whileTrue(Chassis.driveToBarge)
    // y and left bumper
    y().and(leftBumper()).and(notRightBumper).whileTrue(Chassis.driveToBargeLeft)
    // y and right bumper
    y().and(rightBumper()).and(notLeftBumper).whileTrue(Chassis.driveToBargeRight)
}

private fun CommandXboxController.configureDriveManualSequencingLayout() {
    leftTrigger(0.55)
        .onTrue(SuperStructure.goToSelectedLevel)
        .onFalse(SuperStructure.smartGoTo(RobotState.Stow))
    rightTrigger(0.55).whileTrue(Intake.scoreCoral)
    a().whileTrue(Intake.outtakeCoral) // Spencer added, does this work here though?
}

// Driving with buttons for automatic scoring sequences
private fun CommandXboxController.configureDriveAutomaticSequencingLayout() {
    val notLeftBumper = leftBumper().negate()
    val notRightBumper = rightBumper().negate()

    val atApproachPoint = EventTrigger("atApproachPoint")
    // Auto score algae in processor
    b().whileTrue(
            Chassis.driveToProcessor
                .andThen(Intake.dropAlgae)
                .andThen(Chassis.backAwayFromProcessor.until { Chassis.isWithinGoal(0.05) })
                .andThen(SuperStructure.smartGoTo(RobotState.Stow).asProxy())
        )
        // Once at approach point, move to processor position
        .and(atApproachPoint)
        .onTrue(SuperStructure.smartGoTo(RobotState.Processor))

    a().and(notRightBumper)
        .and(notLeftBumper)
        .whileTrue(Chassis.driveToClosestCenterCoralStation.raceWith(LEDSubsystem.flashWhite))
    a().onTrue(
        SuperStructure.smartGoTo(RobotState.NewCoralStation)
            .alongWith(Intake.overIntakeCoralThenHold)
    )

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
    val hasCoral = Trigger { Intake.hasBranchCoral }
    val hasNoCoral = Trigger { !Intake.hasBranchCoral }

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
                Intake.hasBranchCoral
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
                Intake.hasBranchCoral
            }
        )

    leftBumper()
        .and(rightBumper())
        .and(hasNoCoral)
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
            .onlyIf { Intake.detectAlgaeByCurrent() }
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
            Intake.detectAlgaeByCurrent()
        }
    )
}
