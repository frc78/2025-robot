package frc.robot.lib

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.subsystems.Intake

object FieldObstacles {
    private val hasCoral = { Intake.hasBranchCoral }

    private val BLUE_CORAL_STATION_LEFT =
        LineSegment(Translation2d(0.0, 0.0), Translation2d(0.0, 1.0), hasCoral)
    private val BLUE_CORAL_STATION_RIGHT =
        LineSegment(Translation2d(0.0, 0.0), Translation2d(0.0, 1.0), hasCoral)
    private val RED_CORAL_STATION_LEFT =
        LineSegment(Translation2d(0.0, 0.0), Translation2d(0.0, 1.0), hasCoral)
    private val RED_CORAL_STATION_RIGHT =
        LineSegment(Translation2d(0.0, 0.0), Translation2d(0.0, 1.0), hasCoral)

    val CORAL_STATIONS =
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
            listOf(
                BLUE_CORAL_STATION_LEFT,
                BLUE_CORAL_STATION_RIGHT,
            )
        } else {
            listOf(
                RED_CORAL_STATION_LEFT,
                RED_CORAL_STATION_RIGHT,
            )
        }
}

abstract class Obstacle() {
    abstract fun getSlowdownFactor(): Double
}

class LineSegment(val p1: Translation2d, val p2: Translation2d, val applySlow: () -> Boolean) :
    Obstacle() {
    private fun getShortestDistance(): Double {
        TODO("Not yet implemented. SDFs!!!! Tree math!! (Basic tree math)")
    }

    override fun getSlowdownFactor(): Double {
        if (applySlow()) {
            TODO("Not yet implemented")
        }
        return 1.0
    }
}
