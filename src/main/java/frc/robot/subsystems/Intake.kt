package frc.robot.subsystems

import com.ctre.phoenix6.hardware.CANrange
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.lib.centimeters
import org.littletonrobotics.junction.Logger

object Intake : Subsystem {
    private val canRange: CANrange = CANrange(0)

    private val canRangeOffsetEntry =
        NetworkTableInstance.getDefault().getEntry("intake/canRangeOffset")

    private val canRangeOffset
        get() = canRangeOffsetEntry.getDouble(4.0)

    private const val SIDE_PLATE_THICKNESS = 0.5 // Measured in cm.
    private const val INTAKE_WIDTH = 52.0 // Measured in cm.

    val hasCoral: Boolean
        get() = canRange.isDetected.value

    // Returns the distance from the center of the intake to the center of the coral.
    fun coralLocation(): Double {
        if (!hasCoral) {
            return 0.0
        }

        return rawDistance() - (INTAKE_WIDTH / 2) + 5.715
    }

    // Returns the distance from the sensor to the nearest object's edge.
    // Usually returns around +/- 1cm.
    private fun rawDistance(): Double {
        val original = canRange.distance.value.centimeters
        return original - canRangeOffset - SIDE_PLATE_THICKNESS
    }

    override fun periodic() {
        Logger.recordOutput("intake/coral_detected", hasCoral)
        Logger.recordOutput("intake/coral_position", rawDistance())
        Logger.recordOutput("intake/coral_location", coralLocation())
    }
}
