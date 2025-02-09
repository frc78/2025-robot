package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.lib.amps
import frc.robot.lib.centimeters
import frc.robot.lib.command
import org.littletonrobotics.junction.Logger

object Intake : Subsystem {
    private val canRange: CANrange = CANrange(0)

    private val canRangeOffsetEntry =
        NetworkTableInstance.getDefault().getEntry("intake/canRangeOffset")

    private val canRangeOffset
        get() = canRangeOffsetEntry.getDouble(4.0)

    private const val SIDE_PLATE_THICKNESS = 0.5 // Measured in cm.
    private const val INTAKE_WIDTH = 52.0 // Measured in cm.

    private val coralIntake =
        TalonFX(13).apply {
            configurator.apply(
                TalonFXConfiguration().apply {
                    CurrentLimits.StatorCurrentLimit = 40.0
                    CurrentLimits.SupplyCurrentLimit = 20.0
                }
            )
        }
    private val algaeIntake =
        TalonFX(14).apply {
            configurator.apply(
                TalonFXConfiguration().apply {
                    CurrentLimits.StatorCurrentLimit = 40.0
                    CurrentLimits.SupplyCurrentLimit = 20.0
                }
            )
        }

    /**
     * Returns true if a coral is detected in the path of the CANrange. Only corals that are
     * oriented vertically, and thus able to be scored on one of the reef branches, will be detected
     */
    val hasBranchCoral: Boolean
        get() = canRange.isDetected.value

    // Returns the distance from the center of the intake to the center of the coral.
    fun coralLocation(): Double {
        if (!hasBranchCoral) {
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
        Logger.recordOutput("intake/coral_detected", hasBranchCoral)
        Logger.recordOutput("intake/coral_position", rawDistance())
        Logger.recordOutput("intake/coral_location", coralLocation())
    }

    val intakeCoral by command {
        startEnd({ coralIntake.set(0.3) }, { coralIntake.set(0.0) })
            .raceWith(Commands.waitUntil { hasBranchCoral }.andThen(Commands.waitSeconds(0.25)))
            .withName("Intake Coral")
    }
    val intakeAlgae by command {
        startEnd({ algaeIntake.set(0.3) }, { algaeIntake.set(0.0) })
            .until { algaeIntake.torqueCurrent.value > 20.amps }
            .withName("Intake Algae")
    }

    init {
        SmartDashboard.putData(intakeAlgae)
        SmartDashboard.putData(intakeCoral)
    }
}
