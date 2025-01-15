// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.lib.Drive
import frc.robot.subsystems.Chassis

object Robot : TimedRobot() {
    val swerveRequest: SwerveRequest.ApplyFieldSpeeds =
        SwerveRequest.ApplyFieldSpeeds().withDesaturateWheelSpeeds(true)
    val driveController: XboxController = XboxController(0)

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun teleopInit() {
        CommandScheduler.getInstance().cancelAll()
        Chassis.defaultCommand =
            Chassis.applyRequest {
                swerveRequest.withSpeeds(Drive.calculateSpeeds(driveController))
            }
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }
}
