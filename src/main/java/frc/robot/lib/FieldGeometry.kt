package frc.robot.lib

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation

/** Object for defining field geometry and functions to measure distances */
object FieldGeometry {
    private const val FIELD_X_LENGTH = 17.530

    // These are constructed so that left hand perpendicular is always pointing towards the center
    // of the field
    private val BLUE_CORAL_STATION_LOWER =
        LineSegment(Translation2d(0.398, 0.985), Translation2d(1.36, 0.291))
    private val BLUE_CORAL_STATION_UPPER =
        LineSegment(Translation2d(1.373, 7.757), Translation2d(0.409, 7.065))
    private val RED_CORAL_STATION_LOWER =
        LineSegment(BLUE_CORAL_STATION_LOWER.p2.mirror(), BLUE_CORAL_STATION_LOWER.p1.mirror())
    private val RED_CORAL_STATION_UPPER =
        LineSegment(BLUE_CORAL_STATION_UPPER.p2.mirror(), BLUE_CORAL_STATION_UPPER.p1.mirror())

    private val CORAL_STATIONS =
        if (
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
                DriverStation.Alliance.Blue
        ) {
            listOf(BLUE_CORAL_STATION_LOWER, BLUE_CORAL_STATION_UPPER)
        } else {
            listOf(RED_CORAL_STATION_LOWER, RED_CORAL_STATION_UPPER)
        }

    fun distanceToClosestCoralStation(position: Translation2d): Double {
        return CORAL_STATIONS.minOfOrNull { it.getShortestDistance(position) } ?: Double.MAX_VALUE
    }

    fun getClosestCoralStation(position: Translation2d): LineSegment {
        return CORAL_STATIONS.minBy { it.getShortestDistance(position) }
    }

    // There is probably a cleaner way to do this
    private fun Translation2d.mirror(): Translation2d {
        return Translation2d(FIELD_X_LENGTH - x, y)
    }
}
