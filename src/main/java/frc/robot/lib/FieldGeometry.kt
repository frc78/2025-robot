package frc.robot.lib

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.subsystems.Intake

object FieldGeometry {
    private val hasCoral = { Intake.hasBranchCoral }

    private const val FIELD_X_LENGTH = 17.530

    private val BLUE_CORAL_STATION_LOWER =
        LineSegment(Translation2d(0.0, 1.207), Translation2d(1.711, 0.0), hasCoral)
    private val BLUE_CORAL_STATION_UPPER =
        LineSegment(Translation2d(0.0, 0.0), Translation2d(0.0, 1.0), hasCoral)
    private val RED_CORAL_STATION_LOWER =
        LineSegment(BLUE_CORAL_STATION_LOWER.p1.mirror(), BLUE_CORAL_STATION_LOWER.p2.mirror(), hasCoral)
    private val RED_CORAL_STATION_UPPER =
        LineSegment(BLUE_CORAL_STATION_UPPER.p1.mirror(), BLUE_CORAL_STATION_UPPER.p2.mirror(), hasCoral)

    val CORAL_STATIONS =
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
            listOf(BLUE_CORAL_STATION_LOWER, BLUE_CORAL_STATION_UPPER)
        } else {
            listOf(RED_CORAL_STATION_LOWER, RED_CORAL_STATION_UPPER)
        }

    fun distanceToClosestCoralStation(position: Translation2d): Double {
        return CORAL_STATIONS.minOfOrNull { it.getShortestDistance(position) } ?: Double.MAX_VALUE
    }

    // There is probably a cleaner way to do this
    private fun Translation2d.mirror(): Translation2d {
        return Translation2d(FIELD_X_LENGTH - x, y)
    }
}