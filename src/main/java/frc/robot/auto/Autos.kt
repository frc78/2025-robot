package frc.robot.auto

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.Alignments
import frc.robot.lib.command
import frc.robot.subsystems.drivetrain.Chassis

object Autos {
    val FourCoralAuto by command {
        val lineToIJ = PathPlannerPath.fromPathFile("AutoLine to IJ")
        val ijToCoral = PathPlannerPath.fromPathFile("IJToCoral")
        Commands.sequence(
            AutoBuilder.followPath(lineToIJ),
            AutoBuilder.followPath(ijToCoral),
            Chassis.driveToPose { Alignments.closestCoralStation },
            Chassis.driveToPose { Alignments.ReefFace.KL.pose },
        )
    }
}
