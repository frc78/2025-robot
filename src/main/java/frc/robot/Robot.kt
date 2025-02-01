// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix6.swerve.SwerveRequest
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.lib.calculateSpeeds
import frc.robot.subsystems.Chassis
import frc.robot.subsystems.Vision
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGWriter

object Robot : LoggedRobot() {
    val swerveRequest: SwerveRequest.ApplyFieldSpeeds =
        SwerveRequest.ApplyFieldSpeeds().withDesaturateWheelSpeeds(true)
    val driveController: XboxController = XboxController(0)

    val autoChooser = AutoBuilder.buildAutoChooser();

    override fun robotInit() {
        Logger.recordMetadata("ProjectName", "MyProject")
        Logger.addDataReceiver(WPILOGWriter())
        // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(NT4Publisher())
        // Publish data to NetworkTables
        PowerDistribution(1, PowerDistribution.ModuleType.kCTRE)
        Logger.start()

        Chassis.applyRequest { SwerveRequest.Idle() }
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        Vision.update()
        Logger.recordOutput("ChassisPose", Chassis.state.Pose)
    }

    override fun teleopInit() {
        CommandScheduler.getInstance().cancelAll()
        Chassis.defaultCommand =
            Chassis.applyRequest { swerveRequest.withSpeeds(driveController.calculateSpeeds()) }
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun autonomousInit() {
        CommandScheduler.getInstance().cancelAll()
        CommandScheduler.getInstance().schedule(autoChooser.selected)
    }
}
