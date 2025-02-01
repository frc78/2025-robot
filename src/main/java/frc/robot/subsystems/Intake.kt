package frc.robot.subsystems

import com.ctre.phoenix6.hardware.CANrange
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Subsystem

object Intake : Subsystem {
    val canRange: CANrange = CANrange(0)

    // TODO: Change this value for actual robot.
    val sidePlateThickness = 1.25 // Measured in cm.

    var hasCoral: Boolean = false

    fun hasCoral(): Boolean {
        return canRange.getIsDetected().value
    }

    // Returns the distance from the sensor to the nearest object's edge.
    // Usually returns around +/- 1cm.
    fun distance(): Double {
        var original = canRange.getDistance().value.`in`(Units.Centimeter)

        return original - offsetError(original) - sidePlateThickness
    }

    // Adjusts the distance value from the sensor to counter its error.
    private fun offsetError(original: Double): Double {
        return -0.0014 * (original * original) + (0.0854 * original) + 2.9723
    }
}
