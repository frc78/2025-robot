package frc.robot.lib.bindings

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.auto.Autos
import frc.robot.lib.velocityRot
import frc.robot.lib.velocityX
import frc.robot.lib.velocityY
import frc.robot.subsystems.Intake
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
    }
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

    // Removed per driver request 10:00am 3/14
    //    // only a
    //
    // a().and(notRightBumper).and(notLeftBumper).whileTrue(Chassis.driveToClosestCenterCoralStation)
    //    // a and left bumper
    //
    // a().and(leftBumper()).and(notRightBumper).whileTrue(Chassis.driveToClosestLeftCoralStation)
    //    // a and right bumper
    //
    // a().and(rightBumper()).and(notLeftBumper).whileTrue(Chassis.driveToClosestRightCoralStation)
    //    a().onTrue(
    //
    // SuperStructure.smartGoTo(RobotState.CoralStation).alongWith(Intake.intakeCoralThenHold())
    //    )

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
    val notA = a().negate()
    val notY = y().negate()

    b().whileTrue(Chassis.driveToProcessor)

    rightBumper()
        .and(notLeftBumper)
        .whileTrue(
            Chassis.driveToRightBranchAndMoveSuperStucture
            .andThen(SuperStructure.scoreCoralOnSelectedBranch))
    leftBumper()
        .and(notRightBumper)
        .whileTrue(
            Chassis.driveToLeftBranchAndMoveSuperStucture
            .andThen(SuperStructure.scoreCoralOnSelectedBranch))
    leftBumper()
        .and(rightBumper())
        .and { !Intake.hasBranchCoral }
        .whileTrue(
            Chassis.driveToClosestReef.alongWith(SuperStructure.retrieveAlgaeFromReef)
//                .finallyDo { _ -> SuperStructure.retractWithAlgae() }
        )

    a().and(notRightBumper).and(notLeftBumper).whileTrue(Chassis.driveToClosestCenterCoralStation)
    a().onTrue(
        SuperStructure.smartGoTo(RobotState.CoralStation)
        .alongWith(Intake.intakeCoralThenHold()))

    // only y
    y().and(notLeftBumper)
        .and(notRightBumper)
        .whileTrue(Chassis.driveToBarge.andThen(SuperStructure.autoScoreAlgaeInNet))
    // y and left bumper
    y().and(leftBumper())
        .and(notRightBumper)
        .whileTrue(Chassis.driveToBargeLeft.andThen(SuperStructure.autoScoreAlgaeInNet))
    // y and right bumper
    y().and(rightBumper())
        .and(notLeftBumper)
        .whileTrue(Chassis.driveToBargeRight.andThen(SuperStructure.autoScoreAlgaeInNet))
}
