package frc.robot.lib.bindings

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.lib.velocityRot
import frc.robot.lib.velocityX
import frc.robot.lib.velocityY
import frc.robot.subsystems.Intake
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.drivetrain.Chassis

private val DRIVE_LAYOUT =
    DriveLayout.SNAPPING.also { SmartDashboard.putString("drive_layout", it.name) }

val DRIVE_MODIFIERS =
    DriveModifiers(triggerAdjust = true, distanceSlowing = true).also {
        SmartDashboard.putBoolean("trigger_adjust", it.triggerAdjust)
        SmartDashboard.putBoolean("distance_slowing", it.distanceSlowing)
    }

/**
 * Stores values for whether to apply trigger speed, distance slowing, and any other future speed
 * modifiers
 */
class DriveModifiers(val triggerAdjust: Boolean = false, val distanceSlowing: Boolean = false)

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
}

// Driving with snapping bindings for reef alignment
private fun CommandXboxController.configureDriveSnappingLayout() {
    leftBumper().and(rightBumper().negate()).whileTrue(Chassis.driveToLeftBranch)
    rightBumper().and(leftBumper().negate()).whileTrue(Chassis.driveToRightBranch)
    leftBumper().and(rightBumper()).whileTrue(Chassis.driveToClosestReef)

    x().whileTrue(Chassis.snapToReef { withVelocityX(hid.velocityX).withVelocityY(hid.velocityY) })
    y().whileTrue(
        Chassis.snapToClosestSubstation().withName("Snap to closest substation").also {
            SmartDashboard.putData("Snap to closest substation", it)
        }
    )
}

private fun CommandXboxController.configureDriveManualSequencingLayout() {
    leftTrigger(0.55)
        .onTrue(SuperStructure.goToSelectedLevel)
        .onFalse(SuperStructure.smartGoTo(RobotState.Stow))
    rightTrigger(0.55).onTrue(Intake.scoreCoral).onFalse(Intake.defaultCommand)
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
