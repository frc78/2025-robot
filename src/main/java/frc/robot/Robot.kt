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
import edu.wpi.first.wpilibj.GenericHID
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
import frc.robot.subsystems.*
import frc.robot.subsystems.drivetrain.Chassis
import frc.robot.subsystems.drivetrain.Telemetry
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import kotlin.time.Duration.Companion.seconds

// Might have to be manually set when testing on SkibJr
val IS_TEST = "TEST" == System.getenv("frc_bot")
val IS_COMP = "COMP" == System.getenv("frc_bot")

object Robot : LoggedRobot() {
    val gameField: AprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
    val alliance: DriverStation.Alliance
        get() = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)

    init {
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
        Intake
        Pivot
        Wrist

        CommandXboxController(0).configureDriverBindings()
        CommandJoystick(5).configureManipTestBindings()
        CommandXboxController(1).configureManipulatorBindings()

        Pivot.coast()
        RobotModeTriggers.disabled()
            .and(Trigger({ Pivot.angle > 45.degrees }))
            .onTrue(Commands.runOnce({ Pivot.brake() }).ignoringDisable(true))

        // Move wrist over when leaving coral station area with a coral
        Trigger {
            FieldGeometry.distanceToClosestLine(
                FieldGeometry.CORAL_STATIONS,
                Chassis.state.Pose.translation).meters > 1.meters
                    && Intake.hasBranchCoral}.onTrue(SuperStructure.smartGoTo(RobotState.CoralStorage))

        // Rumble for short duration on game piece acquisition
        Trigger { Intake.hasCoralByCurrent() }
            .onTrue(Commands.startEnd(
                { CommandXboxController(0).setRumble(GenericHID.RumbleType.kBothRumble, 1.0) },
                { CommandXboxController(0).setRumble(GenericHID.RumbleType.kBothRumble, 0.0)})
                .withTimeout(0.5))

        Trigger { Intake.detectAlgaeByCurrent() }
            .onTrue(Commands.startEnd(
                { CommandXboxController(0).setRumble(GenericHID.RumbleType.kBothRumble, 1.0) },
                { CommandXboxController(0).setRumble(GenericHID.RumbleType.kBothRumble, 0.0)})
                .withTimeout(0.5))

        // Sets the Wrist to immediately go to its lower limit.  It starts all the way down to zero
        // it,
        // but the lowest safe limit is greater than this due to the top elevator supports
        Wrist.initializePosition()
        PathfindingCommand.warmupCommand().schedule()
    }

    private val autoChooser =
        SendableChooser<Command>().apply {
            setDefaultOption("Four Coral", Autos.FourCoralAuto)
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
                MechanismLigament2d("wrist", 20.0, 90.0, 6.0, Color8Bit(Color.kPurple))
            )
        wristMech.append(MechanismLigament2d("coral", 6.25, 112.0, 6.0, Color8Bit(Color.kWhite)))
        wristMech.append(MechanismLigament2d("algae", 16.0, -125.0, 6.0, Color8Bit(Color.kTeal)))

        // Put the mechanism widget with all its components on the dashboard,
        SmartDashboard.putData("robot", robot)
        //        Vision.setupSimulation()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        if (isReal()) {
            Vision.update()
        }
        SmartDashboard.putBoolean("has coral", Intake.hasCoralByCurrent())
    }

    override fun simulationPeriodic() {
        // The angle of the elevator is determined by the pivot angle
        // The angle parameter expects a value in degrees, so we convert it to degrees
        elevatorMech.angle = Pivot.angle.degrees
        // Elevator.length is 0 when the elevator is retracted, but the elevator has a fixed length
        // of 30 inches
        elevatorMech.length = (30.inches + Elevator.position).inches
        wristMech.angle = -Wrist.angle.degrees
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
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun autonomousInit() {
        CommandScheduler.getInstance().cancelAll()
        autoChooser.selected?.let { CommandScheduler.getInstance().schedule(it) }
    }
}
