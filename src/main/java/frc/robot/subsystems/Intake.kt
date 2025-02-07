package frc.robot.subsystems

import com.ctre.phoenix6.hardware.CANrange
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Subsystem

object Intake : Subsystem {
    private val canRange: CANrange = CANrange(0)

    private const val SIDE_PLATE_THICKNESS = 0.5 // Measured in cm.
    private const val INTAKE_WIDTH = 52.0 // Measured in cm.

    fun hasCoral(): Boolean {
        return canRange.isDetected.value
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
    private fun rawDistance(): Double {
        val original = canRange.distance.value.`in`(Units.Centimeter)

        return original - offsetError(original) - sidePlateThickness
    }

    // Adjusts the distance value from the sensor to counter its error.
    private fun offsetError(original: Double): Double {
        return 4.0
    }
}
