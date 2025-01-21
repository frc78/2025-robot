package frc.robot.subsystems

import edu.wpi.first.math.geometry.Transform3d

object Vision {
    // TW: The naming convention for swerve is F/B for front/back and L/R for left/right
    // TW: So these would be named FL, FR
    val cams: List<Camera> = listOf(Camera("LU", Transform3d()), Camera("RU", Transform3d()))
}
