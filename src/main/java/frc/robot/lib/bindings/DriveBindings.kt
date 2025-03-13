package frc.robot.lib.bindings

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.velocityRot
import frc.robot.lib.velocityX
import frc.robot.lib.velocityY
import frc.robot.subsystems.Intake
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.drivetrain.Chassis

private val DRIVE_LAYOUT =
    DriveLayout.SNAPPING.also { SmartDashboard.putString("drive_layout", it.name) }

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
private fun CommandXboxController.configureDriveBasicLayout() {
    Chassis.defaultCommand =
        Chassis.fieldCentricDrive {
                withVelocityX(hid.velocityX)
                    .withVelocityY(hid.velocityY)
                    .withRotationalRate(hid.velocityRot)
            }
            .withName("Field centric xbox drive")
}

fun only(button: XboxController.Button) =
    DriverStation.getStickButtons(0) xor 1 shl button.value == 0

// Driving with snapping bindings for reef alignment
private fun CommandXboxController.configureDriveSnappingLayout() {
    Trigger {
        DriverStation.getStickButtons(0) xor 1 shl XboxController.Button.kLeftBumper.value == 0
    }
    // only left bumper
    leftBumper()
        .and(rightBumper().negate())
        .and(a().negate())
        .and(y().negate())
        .whileTrue(Chassis.driveToLeftBranch)
    // only right bumper
    rightBumper()
        .and(leftBumper().negate())
        .and(a().negate())
        .and(y().negate())
        .whileTrue(Chassis.driveToRightBranch)

    // both left and right bumper
    leftBumper()
        .and(rightBumper())
        .and(a().negate())
        .and(y().negate())
        .whileTrue(Chassis.driveToClosestReef)

    // only a
    a().and(rightBumper().negate())
        .and(leftBumper().negate())
        .whileTrue(Chassis.driveToClosestCenterCoralStation)
    // a and left bumper
    a().and(leftBumper())
        .and(rightBumper().negate())
        .whileTrue(Chassis.driveToClosestLeftCoralStation)
    // a and right bumper
    a().and(rightBumper())
        .and(leftBumper().negate())
        .whileTrue(Chassis.driveToClosestRightCoralStation)

    x().whileTrue(
        Chassis.snapAngleToReef { withVelocityX(hid.velocityX).withVelocityY(hid.velocityY) }
    )

    // only y
    y().and(leftBumper().negate().and(rightBumper().negate())).whileTrue(Chassis.driveToBarge)
    // y and left bumper
    y().and(leftBumper()).and(rightBumper().negate()).whileTrue(Chassis.driveToBargeLeft)
    // y and right bumper
    y().and(rightBumper()).and(leftBumper().negate()).whileTrue(Chassis.driveToBargeRight)

    b().whileTrue(Chassis.driveToProcessor)
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
    rightBumper()
        .whileTrue(
            Chassis.driveToSelectedBranch
                .andThen(SuperStructure.goToSelectedLevel)
                .andThen(Intake.scoreCoral)
                .andThen(SuperStructure.smartGoTo(RobotState.Stow))
        )
}
