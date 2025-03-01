package frc.robot.lib

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.numbers.N2

abstract class Primitive() {
    abstract fun getVectorToClosestPoint(position: Translation2d): Translation2d

    abstract fun getShortestDistance(position: Translation2d): Double
}

class LineSegment(val p1: Translation2d, val p2: Translation2d, val applySlow: () -> Boolean) :
    Primitive() {
    override fun getVectorToClosestPoint(position: Translation2d): Translation2d {
        // Source: Inigo Quilez https://iquilezles.org/articles/distfunctions2d/
        val positionToP1: Vector<N2> = position.toVector().minus(p1.toVector())
        val p2ToP1: Vector<N2> = p2.toVector().minus(p1.toVector())

        val h = MathUtil.clamp(positionToP1.dot(p2ToP1) / p2ToP1.dot(p2ToP1), 0.0, 1.0)
        val vec = positionToP1.minus(p2ToP1.times(h))
        return Translation2d(vec[0], vec[1])
    }

    override fun getShortestDistance(position: Translation2d): Double {
        return getVectorToClosestPoint(position).norm
    }
}
