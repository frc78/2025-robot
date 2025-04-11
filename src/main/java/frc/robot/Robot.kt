// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix6.SignalLogger
import com.pathplanner.lib.commands.PathfindingCommand
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.auto.Autos
import frc.robot.lib.FieldGeometry
import frc.robot.lib.bindings.configureDriverBindings
import frc.robot.lib.bindings.configureManipTestBindings
import frc.robot.lib.bindings.configureManipulatorBindings
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.lib.meters
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.LEDSubsystem
import frc.robot.subsystems.Pivot
import frc.robot.subsystems.RobotState
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.Vision
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.drivetrain.Chassis
import frc.robot.subsystems.drivetrain.Telemetry
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher

// Might have to be manually set when testing on SkibJr
val IS_COMP = "COMP" == System.getenv("frc_bot")

object Robot : LoggedRobot() {
    val gameField: AprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
    val reefOnlyField: AprilTagFieldLayout =
        AprilTagFieldLayout(Filesystem.getDeployDirectory().path + "/2025-andymark-only-reef.json")
    val alliance: DriverStation.Alliance
        get() = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)

    val driverController = CommandXboxController(0)

    init {
        gameField.tags
        HAL.report(
            FRCNetComm.tResourceType.kResourceType_Language,
            FRCNetComm.tInstances.kLanguage_Kotlin,
            0,
            WPILibVersion.Version,
        )
        DriverStation.silenceJoystickConnectionWarning(true)
        Logger.recordMetadata("IS_COMP", "$IS_COMP")
        // Publish data to NetworkTables
        Logger.addDataReceiver(NT4Publisher())
        PowerDistribution(1, PowerDistribution.ModuleType.kRev)
        Logger.start()
        if (isReal()) {
            // DataLog will automatically log all NT changes. AKit logs to NT, DataLog logs NT to
            // file
            DataLogManager.start()
            SignalLogger.setPath("/U/ctre-logs")
            SignalLogger.start()
        }
        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog())
        Chassis.configureAutoBuilder()
        Chassis.registerTelemetry(Telemetry::telemeterize)

        // Initializing Subsystems
        SuperStructure
        LEDSubsystem
        Intake
        Pivot
        Wrist
        Climber

        driverController.configureDriverBindings()
        CommandXboxController(1).configureManipulatorBindings()
        CommandJoystick(5).configureManipTestBindings()

        Pivot.coast()
        RobotModeTriggers.disabled()
            .and(Trigger { Pivot.angle > 45.degrees })
            .onTrue(Commands.runOnce({ Pivot.brake() }).ignoringDisable(true))

        // Move wrist over when leaving coral station area with a coral
        // Running only in teleop to avoid interrupting auto
        RobotModeTriggers.teleop().and {
                FieldGeometry.distanceToClosestLine(
                        FieldGeometry.CORAL_STATIONS,
                        Chassis.state.Pose.translation,
                    )
                    .meters > 1.meters
            }
            .onTrue(
                Commands.either(
                    Wrist.goToWithoutRequiring(RobotState.CoralStorage),
                    Commands.none(),
                ) {
                    Intake.hasBranchCoral
                }
            )

        RobotModeTriggers.autonomous().and {
            FieldGeometry.distanceToClosestLine(
                FieldGeometry.CORAL_STATIONS,
                Chassis.state.Pose.translation,
            )
                .meters > 0.6.meters
        }
            .onTrue(
                Commands.either(
                    Commands.sequence(
                        Wrist.goToWithoutRequiring(RobotState.CoralStorage),
                        Pivot.goToWithoutRequiring(RobotState.L4)
                    ),
                    Commands.none(),
                ) {
                    Intake.hasBranchCoral
                }
            )

        // Sets the Wrist to immediately go to its lower limit.  It starts all the way down to zero
        // it,
        // but the lowest safe limit is greater than this due to the top elevator supports
        Wrist.initializePosition()
        PathfindingCommand.warmupCommand().schedule()
    }

    private val autoChooser =
        SendableChooser<Command>().apply {
            addOption("Beast Mode 😎", Autos.SideCoralFast)
            addOption("Ball Up Top", Autos.CenterAlgaeAuto)
            SmartDashboard.putData("Auto Mode", this)
        }

    /* lateinit is a way to tell the compiler that we promise to initialize this variable before
    using them. These are lateinit since we don't want to create them always, but when we access them in
    simulationPeriodic we can be confident they were created in simulationInit */
    private lateinit var elevatorMech: MechanismLigament2d
    private lateinit var wristMech: MechanismLigament2d

    override fun simulationInit() {
        // Create a mechanism widget
        val robot = Mechanism2d(150.0, 100.0)
        // The pivot axis location is the root of the mechanism
        // This puts it at 5 units from the left and 5 units from the bottom of the widget
        val pivot = robot.getRoot("pivot", 75.0, 5.0)
        // The elevator gets attached to the pivot. It starts with a length of 30, and an angle of 4
        // degrees
        elevatorMech =
            pivot.append(MechanismLigament2d("elevator", 30.0, 45.0, 4.0, Color8Bit(Color.kOrange)))
        // The wrist gets attached to the elevator. It starts with a length of 20, and an angle of
        // 90.
        // This angle is relative to the elevator, so it will point straight up when the elevator is
        // at 0º, and will point
        // to the left when the elevator is at 90º
        wristMech =
            elevatorMech.append(
                MechanismLigament2d("wrist", 12.0, 90.0, 6.0, Color8Bit(Color.kPurple))
            )
        wristMech.append(MechanismLigament2d("coral", 6.25, 112.0, 6.0, Color8Bit(Color.kWhite)))

        // Put the mechanism widget with all its components on the dashboard,
        SmartDashboard.putData("robot", robot)
        //        Vision.setupSimulation()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        if (isReal()) {
            Vision.update()
        }
    }

    override fun simulationPeriodic() {
        // The angle of the elevator is determined by the pivot angle
        // The angle parameter expects a value in degrees, so we convert it to degrees
        elevatorMech.angle = Pivot.angle.degrees
        // Elevator.length is 0 when the elevator is retracted, but the elevator has a fixed length
        // of 30 inches
        elevatorMech.length = (30.inches + Elevator.position).inches
        wristMech.angle = 140 - Wrist.angle.degrees
    }

    override fun teleopInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun teleopExit() {
        // Stop logging at the end of the match
        if (DriverStation.isFMSAttached()) {
            Logger.end()
            DataLogManager.stop()
            SignalLogger.stop()
        }
        Pivot.brake()
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun autonomousInit() {
        CommandScheduler.getInstance().cancelAll()
        autoChooser.selected?.let { CommandScheduler.getInstance().schedule(it) }
    }
}
