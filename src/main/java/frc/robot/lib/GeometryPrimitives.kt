package frc.robot.lib

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.numbers.N2

abstract class Primitive {
    /** Returns vector from the closest point of primitive to the input position */
    abstract fun getVectorFromClosestPoint(position: Translation2d): Translation2d

    /** Gets the shortest distance to the primitive */
    abstract fun getShortestDistance(position: Translation2d): Double
}

class LineSegment(val p1: Translation2d, val p2: Translation2d) : Primitive() {
    override fun getVectorFromClosestPoint(position: Translation2d): Translation2d {
        // Source: Inigo Quilez https://iquilezles.org/articles/distfunctions2d/
        val positionToP1: Vector<N2> = position.toVector().minus(p1.toVector())
        val p2ToP1: Vector<N2> = p2.toVector().minus(p1.toVector())

        val h = MathUtil.clamp(positionToP1.dot(p2ToP1) / p2ToP1.dot(p2ToP1), 0.0, 1.0)
        val vec = positionToP1.minus(p2ToP1.times(h))
        return Translation2d(vec[0], vec[1])
    }

    override fun getShortestDistance(position: Translation2d): Double {
        return getVectorFromClosestPoint(position).norm
    }

    /** Unit vector pointing to left hand (looking from p1 to p2) perpendicular direction */
    fun getPerpendicularUnitVector(): Translation2d {
        val p2ToP1 = p2.minus(p1)
        return Translation2d(-p2ToP1.y, p2ToP1.x).div(p2ToP1.norm)
    }

    /** Get the parallel unit vector pointing from p2 to p1 */
    fun getParallelUnitVector(): Translation2d {
        val p2ToP1 = p2.minus(p1)
        return p2ToP1.div(p2ToP1.norm)
    }
}
