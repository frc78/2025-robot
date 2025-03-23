package frc.robot.lib

import edu.wpi.first.math.MathSharedStore
import edu.wpi.first.math.MathUsageId
import java.util.*
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

class AsymtrapezoidalProfile
/**
 * Constructs an Asymmetrical Trapezoidal Profile.
 *
 * @param constraints The constraints on the profile, including maximum velocity, acceleration, and deceleration.
 */(private val constraints: Constraints) {
    // The direction of the profile, either 1 for forwards or -1 for inverted
    private var direction = 0

    private var currrent: State? = null

    private var endAccel = 0.0
    private var endFullSpeed = 0.0
    private var endDecel = 0.0

    /** Profile constraints.  */
    class Constraints(
        /** Maximum velocity.  */
        val maxVelocity: Double,
        /** Maximum acceleration.  */
        val maxAcceleration: Double,
        /** Maximum deceleration.  */
        val maxDeceleration: Double
    ) {
        /**
         * Constructs constraints for an Asymmetrical Trapezoidal Profile.
         *
         * @param maxVelocity maximum velocity
         * @param maxAcceleration maximum acceleration
         * @param maxDeceleration maximum deceleration
         */
        init {
            MathSharedStore.reportUsage(MathUsageId.kTrajectory_TrapezoidProfile, 1)
        }
    }

    /** Profile state.  */
    class State
    /**
     * Constructs constraints for a Trapezoid Profile.
     *
     * @param position The position at this state.
     * @param velocity The velocity at this state.
     */(
        /** The position at this state.  */
        var position: Double = 0.0,
        /** The velocity at this state.  */
        var velocity: Double = 0.0
    ) {

        override fun equals(other: Any?): Boolean {
            return other is State
                    && this.position == other.position && this.velocity == other.velocity
        }

        override fun hashCode(): Int {
            return Objects.hash(position, velocity)
        }
    }

    /**
     * Calculates the position and velocity for the profile at a time t where the current state is at
     * time t = 0.
     *
     * @param t How long to advance from the current state toward the desired state.
     * @param current The current state.
     * @param goal The desired state when the profile is complete.
     * @return The position and velocity of the profile at time t.
     */
    fun calculate(t: Double, current: State, goal: State): State {
        var goal = goal
        direction = if (shouldFlipAcceleration(current, goal)) -1 else 1
        currrent = direct(current)
        goal = direct(goal)

        if (currrent!!.velocity > constraints.maxVelocity) {
            currrent!!.velocity = constraints.maxVelocity
        }

        // Deal with a possibly truncated motion profile (with nonzero initial or
        // final velocity) by calculating the parameters as if the profile began and
        // ended at zero velocity
        val cutoffBegin = currrent!!.velocity / constraints.maxAcceleration
        val cutoffDistBegin = cutoffBegin.squared() * constraints.maxAcceleration / 2.0

        val cutoffEnd = goal.velocity / constraints.maxDeceleration
        val cutoffDistEnd = cutoffEnd.squared() * constraints.maxDeceleration / 2.0

        // Now we can calculate the parameters as if it was a full trapezoid instead
        // of a truncated one
        val fullTrapezoidDist =
            cutoffDistBegin + (goal.position - currrent!!.position) + cutoffDistEnd
        var accelerationTime = constraints.maxVelocity / constraints.maxAcceleration
        var decelerationTime = constraints.maxVelocity / constraints.maxDeceleration

        var fullSpeedDist =
            fullTrapezoidDist - (accelerationTime.squared() * constraints.maxAcceleration / 2.0) -
                    (decelerationTime.squared() * constraints.maxDeceleration / 2.0)

        // Handle the case where the profile never reaches full speed
        if (fullSpeedDist < 0) {
            // Solve for the peak velocity that is attainable
            val peakVel = sqrt(
                (2.0 * constraints.maxAcceleration * constraints.maxDeceleration * fullTrapezoidDist) /
                        (constraints.maxAcceleration + constraints.maxDeceleration)
            )
            accelerationTime = peakVel / constraints.maxAcceleration
            decelerationTime = peakVel / constraints.maxDeceleration
            fullSpeedDist = 0.0
        }

        endAccel = accelerationTime - cutoffBegin
        endFullSpeed = endAccel + fullSpeedDist / constraints.maxVelocity
        endDecel = endFullSpeed + decelerationTime - cutoffEnd
        var result = State(currrent!!.position, currrent!!.velocity)

        if (t < endAccel) {
            result.velocity += t * constraints.maxAcceleration
            result.position += (currrent!!.velocity + t * constraints.maxAcceleration / 2.0) * t
        } else if (t < endFullSpeed) {
            result.velocity = constraints.maxVelocity
            result.position +=
                ((currrent!!.velocity + endAccel * constraints.maxAcceleration / 2.0) * endAccel
                        + constraints.maxVelocity * (t - endAccel))
        } else if (t <= endDecel) {
            result.velocity = goal.velocity + (endDecel - t) * constraints.maxDeceleration
            val timeLeft = endDecel - t
            result.position =
                (goal.position
                        - (goal.velocity + timeLeft * constraints.maxDeceleration / 2.0) * timeLeft)
        } else {
            result = goal
        }

        return direct(result)
    }

    /**
     * Returns the time left until a target distance in the profile is reached.
     *
     * @param target The target distance.
     * @return The time left until a target distance in the profile is reached.
     */
    fun timeLeftUntil(target: Double): Double {
        val position = currrent!!.position * direction
        var velocity = currrent!!.velocity * direction

        var endAccel = endAccel * direction
        var endFullSpeed = endFullSpeed * direction - endAccel

        if (target < position) {
            endAccel = -endAccel
            endFullSpeed = -endFullSpeed
            velocity = -velocity
        }

        endAccel = max(endAccel, 0.0)
        endFullSpeed = max(endFullSpeed, 0.0)

        val acceleration = constraints.maxAcceleration
        val deceleration = -constraints.maxDeceleration

        val distToTarget = abs(target - position)
        if (distToTarget < 1e-6) {
            return 0.0
        }

        var accelDist = velocity * endAccel + 0.5 * acceleration * endAccel * endAccel
        val decelVelocity = if (endAccel > 0) {
            sqrt(abs(velocity * velocity + 2 * acceleration * accelDist))
        } else {
            velocity
        }

        var fullSpeedDist = constraints.maxVelocity * endFullSpeed
        val decelDist: Double

        if (accelDist > distToTarget) {
            accelDist = distToTarget
            fullSpeedDist = 0.0
            decelDist = 0.0
        } else if (accelDist + fullSpeedDist > distToTarget) {
            fullSpeedDist = distToTarget - accelDist
            decelDist = 0.0
        } else {
            decelDist = distToTarget - fullSpeedDist - accelDist
        }

        val accelTime =
            ((-velocity + sqrt(abs(velocity.squared() + 2 * acceleration * accelDist)))
                    / acceleration)

        val decelTime =
            ((-decelVelocity
                    + sqrt(abs(decelVelocity.squared() + 2 * deceleration * decelDist)))
                    / deceleration)

        val fullSpeedTime = fullSpeedDist / constraints.maxVelocity

        return accelTime + fullSpeedTime + decelTime
    }

    /**
     * Returns the total time the profile takes to reach the goal.
     *
     * @return The total time the profile takes to reach the goal.
     */
    fun totalTime(): Double {
        return endDecel
    }

    /**
     * Returns true if the profile has reached the goal.
     *
     *
     * The profile has reached the goal if the time since the profile started has exceeded the
     * profile's total time.
     *
     * @param t The time since the beginning of the profile.
     * @return True if the profile has reached the goal.
     */
    fun isFinished(t: Double): Boolean {
        return t >= totalTime()
    }

    // Flip the sign of the velocity and position if the profile is inverted
    private fun direct(`in`: State): State {
        val result = State(`in`.position, `in`.velocity)
        result.position *= direction
        result.velocity *= direction
        return result
    }

    companion object {
        /**
         * Returns true if the profile inverted.
         *
         *
         * The profile is inverted if goal position is less than the initial position.
         *
         * @param initial The initial state (usually the current state).
         * @param goal The desired state when the profile is complete.
         */
        private fun shouldFlipAcceleration(initial: State, goal: State): Boolean {
            return initial.position > goal.position
        }
    }
}