package frc.robot.lib

import edu.wpi.first.math.MathSharedStore
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import java.lang.IllegalArgumentException

class AsymProfiledPIDController @JvmOverloads constructor(
    kP: Double,
    kI: Double,
    kD: Double,
    constraints: AsymtrapezoidalProfile.Constraints,
    period: Double = 0.02
) :
    Sendable {
    private val controller = PIDController(kP, kI, kD, period)
    private var minimumInput = 0.0
    private var maximumInput = 0.0

    private var m_constraints: AsymtrapezoidalProfile.Constraints
    private var profile: AsymtrapezoidalProfile
    /**
     * Gets the goal for the ProfiledPIDController.
     *
     * @return The goal.
     */
    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal state.
     */
    var goal: AsymtrapezoidalProfile.State = AsymtrapezoidalProfile.State()

    /**
     * Returns the current setpoint of the ProfiledPIDController.
     *
     * @return The current setpoint.
     */
    var setpoint: AsymtrapezoidalProfile.State = AsymtrapezoidalProfile.State()
        private set

    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and Kd.
     *
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     * @param period The period between controller updates in seconds. The default is 0.02 seconds.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     * @throws IllegalArgumentException if period &lt;= 0
     */
    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and Kd.
     *
     * @param Kp The proportional coefficient.
     * @param Ki The integral coefficient.
     * @param Kd The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     */
    init {
        m_constraints = constraints
        profile = AsymtrapezoidalProfile(m_constraints)
        instances++

        SendableRegistry.add(this, "AsymProfiledPIDController", instances)
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     *
     * Sets the proportional, integral, and differential coefficients.
     *
     * @param Kp The proportional coefficient. Must be &gt;= 0.
     * @param Ki The integral coefficient. Must be &gt;= 0.
     * @param Kd The differential coefficient. Must be &gt;= 0.
     */
    fun setPID(Kp: Double, Ki: Double, Kd: Double) {
        controller.setPID(Kp, Ki, Kd)
    }

    var p: Double
        /**
         * Gets the proportional coefficient.
         *
         * @return proportional coefficient
         */
        get() = controller.p
        /**
         * Sets the proportional coefficient of the PID controller gain.
         *
         * @param kP The proportional coefficient. Must be &gt;= 0.
         */
        set(kP) {
            controller.p = kP
        }

    var i: Double
        /**
         * Gets the integral coefficient.
         *
         * @return integral coefficient
         */
        get() = controller.i
        /**
         * Sets the integral coefficient of the PID controller gain.
         *
         * @param kI The integral coefficient. Must be &gt;= 0.
         */
        set(kI) {
            controller.i = kI
        }

    var d: Double
        /**
         * Gets the differential coefficient.
         *
         * @return differential coefficient
         */
        get() = controller.d
        /**
         * Sets the differential coefficient of the PID controller gain.
         *
         * @param kD The differential coefficient. Must be &gt;= 0.
         */
        set(kD) {
            controller.d = kD
        }

    var iZone: Double
        /**
         * Get the IZone range.
         *
         * @return Maximum magnitude of error to allow integral control.
         */
        get() = controller.iZone
        /**
         * Sets the IZone range. When the absolute value of the position error is greater than IZone, the
         * total accumulated error will reset to zero, disabling integral gain until the absolute value of
         * the position error is less than IZone. This is used to prevent integral windup. Must be
         * non-negative. Passing a value of zero will effectively disable integral gain. Passing a value
         * of [Double.POSITIVE_INFINITY] disables IZone functionality.
         *
         * @param iZone Maximum magnitude of error to allow integral control.
         * @throws IllegalArgumentException if iZone &lt;= 0
         */
        set(iZone) {
            controller.iZone = iZone
        }

    val period: Double
        /**
         * Gets the period of this controller.
         *
         * @return The period of the controller.
         */
        get() = controller.period

    val positionTolerance: Double
        /**
         * Returns the position tolerance of this controller.
         *
         * @return the position tolerance of the controller.
         */
        get() = controller.errorTolerance

    val velocityTolerance: Double
        /**
         * Returns the velocity tolerance of this controller.
         *
         * @return the velocity tolerance of the controller.
         */
        get() = controller.errorDerivativeTolerance

    val accumulatedError: Double
        /**
         * Returns the accumulated error used in the integral calculation of this controller.
         *
         * @return The accumulated error of this controller.
         */
        get() = controller.accumulatedError

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal position.
     */
    fun setGoal(goal: Double) {
        this.goal = AsymtrapezoidalProfile.State(goal, 0.0)
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     *
     * This will return false until at least one input value has been computed.
     *
     * @return True if the error is within the tolerance of the error.
     */
    fun atGoal(): Boolean {
        return atSetpoint() && goal == setpoint
    }

    var constraints: AsymtrapezoidalProfile.Constraints
        /**
         * Get the velocity and acceleration constraints for this controller.
         *
         * @return Velocity and acceleration constraints.
         */
        get() = m_constraints
        /**
         * Set velocity and acceleration constraints for goal.
         *
         * @param constraints Velocity and acceleration constraints for goal.
         */
        set(constraints) {
            m_constraints = constraints
            profile = AsymtrapezoidalProfile(m_constraints)
        }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     *
     * This will return false until at least one input value has been computed.
     *
     * @return True if the error is within the tolerance of the error.
     */
    fun atSetpoint(): Boolean {
        return controller.atSetpoint()
    }

    /**
     * Enables continuous input.
     *
     *
     * Rather then using the max and min input range as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    fun enableContinuousInput(minimumInput: Double, maximumInput: Double) {
        controller.enableContinuousInput(minimumInput, maximumInput)
        this.minimumInput = minimumInput
        this.maximumInput = maximumInput
    }

    /** Disables continuous input.  */
    fun disableContinuousInput() {
        controller.disableContinuousInput()
    }

    /**
     * Sets the minimum and maximum contributions of the integral term.
     *
     *
     * The internal integrator is clamped so that the integral term's contribution to the output
     * stays between minimumIntegral and maximumIntegral. This prevents integral windup.
     *
     * @param minimumIntegral The minimum contribution of the integral term.
     * @param maximumIntegral The maximum contribution of the integral term.
     */
    fun setIntegratorRange(minimumIntegral: Double, maximumIntegral: Double) {
        controller.setIntegratorRange(minimumIntegral, maximumIntegral)
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    fun setTolerance(positionTolerance: Double) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY)
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    fun setTolerance(positionTolerance: Double, velocityTolerance: Double) {
        controller.setTolerance(positionTolerance, velocityTolerance)
    }

    val positionError: Double
        /**
         * Returns the difference between the setpoint and the measurement.
         *
         * @return The error.
         */
        get() = controller.error

    val velocityError: Double
        /**
         * Returns the change in error per second.
         *
         * @return The change in error per second.
         */
        get() = controller.errorDerivative

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The controller's next output.
     */
    fun calculate(measurement: Double): Double {
        if (controller.isContinuousInputEnabled) {
            // Get error which is the smallest distance between goal and measurement
            val errorBound = (maximumInput - minimumInput) / 2.0
            val goalMinDistance =
                MathUtil.inputModulus(goal.position - measurement, -errorBound, errorBound)
            val setpointMinDistance =
                MathUtil.inputModulus(setpoint.position - measurement, -errorBound, errorBound)

            // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
            // may be outside the input range after this operation, but that's OK because the controller
            // will still go there and report an error of zero. In other words, the setpoint only needs to
            // be offset from the measurement by the input range modulus; they don't need to be equal.
            goal.position = goalMinDistance + measurement
            setpoint.position = setpointMinDistance + measurement
        }

        setpoint = profile.calculate(period, setpoint, goal)
        return controller.calculate(measurement, setpoint.position)
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal The new goal of the controller.
     * @return The controller's next output.
     */
    fun calculate(measurement: Double, goal: AsymtrapezoidalProfile.State): Double {
        this.goal = goal
        return calculate(measurement)
    }

    /**
     * Returns the next output of the PIDController.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal The new goal of the controller.
     * @return The controller's next output.
     */
    fun calculate(measurement: Double, goal: Double): Double {
        setGoal(goal)
        return calculate(measurement)
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal The new goal of the controller.
     * @param constraints Velocity and acceleration constraints for goal.
     * @return The controller's next output.
     */
    fun calculate(
        measurement: Double, goal: AsymtrapezoidalProfile.State, constraints: AsymtrapezoidalProfile.Constraints
    ): Double {
        this.constraints = constraints
        return calculate(measurement, goal)
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measurement The current measured State of the system.
     */
    fun reset(measurement: AsymtrapezoidalProfile.State) {
        controller.reset()
        setpoint = measurement
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system.
     * @param measuredVelocity The current measured velocity of the system.
     */
    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system. The velocity is assumed to
     * be zero.
     */
    @JvmOverloads
    fun reset(measuredPosition: Double, measuredVelocity: Double = 0.0) {
        reset(AsymtrapezoidalProfile.State(measuredPosition, measuredVelocity))
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType("ProfiledPIDController")
        builder.addDoubleProperty("p", { this.p }, { Kp: Double ->
            this.p =
                Kp
        })
        builder.addDoubleProperty("i", { this.i }, { Ki: Double ->
            this.i =
                Ki
        })
        builder.addDoubleProperty("d", { this.d }, { Kd: Double ->
            this.d =
                Kd
        })
        builder.addDoubleProperty(
            "izone",
            { this.iZone },
            { toSet: Double ->
                try {
                    iZone = toSet
                } catch (e: IllegalArgumentException) {
                    MathSharedStore.reportError("IZone must be a non-negative number!", e.stackTrace)
                }
            })
        builder.addDoubleProperty(
            "maxVelocity",
            { constraints.maxVelocity },
            { maxVelocity: Double ->
                constraints = AsymtrapezoidalProfile.Constraints(maxVelocity, constraints.maxAcceleration, constraints.maxDeceleration)
            })
        builder.addDoubleProperty(
            "maxAcceleration",
            { constraints.maxAcceleration },
            { maxAcceleration: Double ->
                constraints = AsymtrapezoidalProfile.Constraints(constraints.maxVelocity, maxAcceleration, constraints.maxDeceleration)
            })
        builder.addDoubleProperty(
            "maxDeceleration",
            { constraints.maxDeceleration },
            { maxDeceleration: Double ->
                constraints = AsymtrapezoidalProfile.Constraints(constraints.maxVelocity, constraints.maxAcceleration, maxDeceleration)
            })
        builder.addDoubleProperty(
            "goal",
            { goal.position },
            { goal: Double -> this.setGoal(goal) })
    }

    companion object {
        private var instances = 0
    }
}