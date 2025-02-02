package frc.robot.lib

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance

// Tim, does this work?
val REEF_POSITION = if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) Translation2d() else Translation2d()
