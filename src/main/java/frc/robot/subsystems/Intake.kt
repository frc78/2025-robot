package frc.robot.subsystems

import com.ctre.phoenix6.hardware.CANrange
import edu.wpi.first.units.Units.Centimeter
import edu.wpi.first.units.Units.Centimeters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.centimeters

object Intake : SubsystemBase("Intake") {

    private val SIDE_PLATE_THICKNESS = 1.25.centimeters

    private val canRange: CANrange = CANrange(0)

    /**
     * Returns true if a coral is detected in the path of the CANrange. Only corals that are
     * oriented vertically, and thus able to be scored on one of the reef branches, will be detected
     */
    val hasBranchCoral: Boolean
        get() = canRange.isDetected.value

    /**
     * Returns the distance from the sensor to the nearest object's edge. Usually returns around +/-
     * 1cm.
     *
     * Keep in a mutable measure to reduce memory allocations
     */
    private val coralLocationDistanceMutable = Centimeters.mutable(0.0)
    val coralLocation: Distance
        get() {
            val original = canRange.distance.value.centimeters
            val adjusted = original - offsetError(original) - SIDE_PLATE_THICKNESS.centimeters
            return coralLocationDistanceMutable.mut_replace(adjusted, Centimeter)
        }

    // Adjusts the distance value from the sensor to counter its error.
    private fun offsetError(original: Double): Double {
        return -0.0014 * (original * original) + (0.0854 * original) + 2.9723
    }
}
