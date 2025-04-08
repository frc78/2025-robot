package frc.robot.commands

import frc.robot.lib.andWait
import frc.robot.lib.command
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.drivetrain.Chassis

val scoreCoralWhenClose by command {
    SuperStructure.goToScoreCoralWhenClose
        .andWait { Chassis.isStableWithinGoal(0.05) }
        .andThen(SuperStructure.scoreCoralOnSelectedBranch)
}
