package frc.robot.subsystems

import com.ctre.phoenix6.hardware.CANrange
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.lib.centimeters

object Intake : Subsystem {
    val canRange: CANrange = CANrange(0)

    const val sidePlateThickness = 0.5 // Measured in cm.
    const val intakeWidth = 52.0 // Measured in cm.

    val hasCoral: Boolean
        get() = canRange.getIsDetected().value

    // Returns the distance from the center of the intake to the center of the coral.
    fun distance(): Double {
        if (!hasCoral) {
            return 0.0
        }

        return rawDistance() - (intakeWidth / 2) + 5.715
    }

    // Returns the distance from the sensor to the nearest object's edge.
    // Usually returns around +/- 1cm.
    fun rawDistance(): Double {
        val original = canRange.getDistance().value.centimeters

        return original - 4.0 - sidePlateThickness
    }
}
