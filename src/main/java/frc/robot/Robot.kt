// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.auto.Autos
import frc.robot.lib.ScoreSelector
import frc.robot.lib.amps
import frc.robot.lib.configureDriverBindings
import frc.robot.lib.configureManipTestBindings
import frc.robot.lib.configureManipulatorBindings
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Pivot
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.Vision
import frc.robot.subsystems.Wrist
import frc.robot.subsystems.drivetrain.Chassis
import frc.robot.subsystems.drivetrain.Telemetry
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGWriter

// Might have to be manually set when testing on SkibJr
val IS_TEST = "TEST" == System.getenv("frc_bot")

object Robot : LoggedRobot() {
    val gameField: AprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)

    init {
        DriverStation.silenceJoystickConnectionWarning(true)
        Logger.recordMetadata("IS_TEST ?", "$IS_TEST")
        if (isReal()) {
            // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(WPILOGWriter())
        }
        // Publish data to NetworkTables
        Logger.addDataReceiver(NT4Publisher())
        PowerDistribution(1, PowerDistribution.ModuleType.kCTRE)
        Logger.start()
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

        SmartDashboard.putData(
            "Flip driver station",
            Commands.runOnce({
                println("Flipping driver station perspective")
                Chassis.setOperatorPerspectiveForward(
                    if (
                        Chassis.operatorForwardDirection == Chassis.kBlueAlliancePerspectiveRotation
                    )
                        Chassis.kRedAlliancePerspectiveRotation
                    else Chassis.kBlueAlliancePerspectiveRotation
                )
            }),
        )

        // Sets the Wrist to immediately go to its lower limit.  It starts all the way down to zero it,
        // but the lowest safe limit is greater than this due to the top elevator supports
        Wrist.initializePosition()
    }

    private val autoChooser =
        AutoBuilder.buildAutoChooser("test").also {
            it.addOption("FourCoral", Autos.FourCoralAuto)
            SmartDashboard.putData("Auto Mode", it)
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
        Vision.setupSimulation()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        Vision.update()
        ScoreSelector.telemeterize()

        // Should these be in corresponding subsystems?
        Logger.recordOutput("Pivot", Pivot.angle.degrees)
        Logger.recordOutput("Elevator", Elevator.position.inches)
        Logger.recordOutput("Wrist", Wrist.angle.degrees)

        Logger.recordOutput("Ele Stowed", Elevator.isStowed)
        Logger.recordOutput("Intake Current", Intake.supplyCurrent.amps)

        SmartDashboard.putBoolean("Has Algae", Intake.hasAlgaeByCurrent())
        SmartDashboard.putNumber("Intake Current", Intake.torqueCurrent.amps)
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
