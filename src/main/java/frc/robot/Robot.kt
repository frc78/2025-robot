// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Intake
import frc.robot.subsystems.SuperStructure

object Robot : TimedRobot() {

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    init {
        // Initlizing Sub-Systems
        SuperStructure
        Intake
        Drivetrain
        Climber
    }
}
