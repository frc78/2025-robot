package frc.robot.subsystems

import com.ctre.phoenix6.hardware.CANrange
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Subsystem

object Intake : Subsystem {
    val canRange: CANrange = CANrange(0)

    const val sidePlateThickness = 0.5 // Measured in cm.
    const val intakeWidth = 52.0 // Measured in cm.

    var hasCoral: Boolean = false

    fun HasCoral(): Boolean {
        return canRange.getIsDetected().value
    }

    // Returns the distance from the center of the intake to the center of the coral.
    fun distance(): Double {
        if (!HasCoral()) {
            return 0.0
        }

        return rawDistance() - (intakeWidth / 2) + 5.715
    }

    // Returns the distance from the sensor to the nearest object's edge.
    // Usually returns around +/- 1cm.
    fun rawDistance(): Double {
        var original = canRange.getDistance().value.`in`(Units.Centimeter)

        return original - offsetError(original) - sidePlateThickness
    }

    // Adjusts the distance value from the sensor to counter its error.
    private fun offsetError(original: Double): Double {
        return 4.0
    }
}
