package frc.robot.lib

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.numbers.N2
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
            listOf(BLUE_CORAL_STATION_LEFT, BLUE_CORAL_STATION_RIGHT)
        } else {
            listOf(RED_CORAL_STATION_LEFT, RED_CORAL_STATION_RIGHT)
        }
}

abstract class Obstacle() {
    abstract fun getSlowdownFactor(): Double
}

class LineSegment(val p1: Translation2d, val p2: Translation2d, val applySlow: () -> Boolean) :
    Obstacle() {
    private fun getShortestDistance(position: Translation2d): Double {
        // Source: Inigo Quilez https://iquilezles.org/articles/distfunctions2d/
        val positionToP1: Vector<N2> = position.toVector().minus(p1.toVector())
        val p2ToP1: Vector<N2> = p2.toVector().minus(p1.toVector())

        val h = MathUtil.clamp(positionToP1.dot(p2ToP1) / p2ToP1.dot(p2ToP1), 0.0, 1.0)
        return positionToP1.minus(p2ToP1.times(h)).norm()
    }

    override fun getSlowdownFactor(): Double {
        if (applySlow()) {
            TODO("Not yet implemented")
        }
        return 1.0
    }
}

// In progress
private fun quadraticDecrease(
    distance: Double,
    startSlowDistance: Double,
    maxSlowDistance: Double,
    maxSlowdown: Double,
): Double {
    return MathUtil.clamp(
        distance - startSlowDistance,
        0.0,
        maxSlowDistance,
    )
}