package frc.robot.lib

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.drivetrain.Chassis

private val DRIVE_LAYOUT =
    DriveLayout.AUTOMATIC.also { SmartDashboard.putString("drive_layout", it.name) }

enum class DriveLayout {
    BASIC,
    SNAPPING,
    AUTOMATIC,
}

fun CommandXboxController.configureDriverBindings() {
    when (DRIVE_LAYOUT) {
        DriveLayout.BASIC -> configureDriveBasicLayout()
        DriveLayout.SNAPPING -> configureDriveSnappingLayout()
        DriveLayout.AUTOMATIC -> configureDriveAutomaticLayout()
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

    start().onTrue(Commands.runOnce({ Chassis.zeroHeading }))
}

// Driving with snapping bindings for reef alignment
fun CommandXboxController.configureDriveSnappingLayout() {
    configureDriveBasicLayout()

    leftBumper().whileTrue(Chassis.driveToLeftBranch)
    rightBumper().whileTrue(Chassis.driveToRightBranch)
    y().whileTrue(Elevator.manualUp)
    a().whileTrue(Elevator.manualDown)

    x().whileTrue(Chassis.snapToReef { withVelocityX(hid.velocityX).withVelocityY(hid.velocityY) })
}

// Driving with buttons for automatic scoring sequences
fun CommandXboxController.configureDriveAutomaticLayout() {
    configureDriveBasicLayout()

    rightBumper()
        .whileTrue(
            Chassis.driveToSelectedBranch
                .andThen(SuperStructure.goToSelectedLevel)
                .andThen(Intake.scoreCoral)
                .andThen(SuperStructure.smartGoTo(RobotState.Stow))
        )
}
