package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.lib.amps
import frc.robot.lib.centimeters
import frc.robot.lib.command
import org.littletonrobotics.junction.Logger

object Intake : Subsystem {
    init {
        defaultCommand = Commands.idle(this)
    }

    private val canRange: CANrange = CANrange(0, "*")

    private val canRangeOffsetEntry =
        NetworkTableInstance.getDefault().getEntry("intake/canRangeOffset")

    private val canRangeOffset
        get() = canRangeOffsetEntry.getDouble(4.0).centimeters

    private val SIDE_PLATE_THICKNESS = 0.5.centimeters // Measured in cm.
    private val INTAKE_WIDTH = 40.5.centimeters // Measured in cm.

    // TODO determine these values empirically for the new intake - graph on dashboard?
    private val CORAL_CURRENT_THRESHOLD =
        0.amps // Current spike threshold for detecting when we have a coral
    //    private val ALGAE_CURRENT_THRESHOLD =
    //        0.amps // Current spike threshold for detecting when we have an algae
    //
    //    private val coralIntake =
    //        TalonFX(14, "*").apply {
    //            configurator.apply(
    //                TalonFXConfiguration().apply {
    //                    CurrentLimits.StatorCurrentLimit = 40.0
    //                    CurrentLimits.SupplyCurrentLimit = 20.0
    //                }
    //            )
    //        }

    // TODO check id, inversion, etc. before running because of new intake design
    private val leader = // used to be the algae motor, now is the only motor on new rev of intake
        TalonFX(15, "*").apply {
            configurator.apply(
                TalonFXConfiguration().apply {
                    CurrentLimits.StatorCurrentLimit = 40.0
                    CurrentLimits.SupplyCurrentLimit = 20.0
                    MotorOutput.Inverted = InvertedValue.Clockwise_Positive
                }
            )
        }

    init {
        leader.set(0.0)
    }

    /**
     * Returns true if a coral is detected in the path of the CANrange. Only corals that are
     * oriented vertically, and thus able to be scored on one of the reef branches, will be detected
     */
    val hasBranchCoral: Boolean
        get() = canRange.isDetected.value

    val supplyCurrent: Current
        get() = leader.supplyCurrent.value

    /**
     * Return true if the intake motor is experiencing current draw greater than the given
     * threshold.
     */
    fun hasGamePieceByCurrent(threshold: Current): Boolean {
        return leader.supplyCurrent.value >= threshold
    }

    // Returns the distance from the center of the intake to the center of the coral.
    // TODO check this formula with the new intake
    val coralLocation: Distance
        get() {
            if (!hasBranchCoral) {
                return 0.0.centimeters
            }

            return -(rawDistance() - (INTAKE_WIDTH / 2.0) + 3.centimeters)
        }

    // Returns the distance from the sensor to the nearest object's edge.
    // Usually returns around +/- 1cm.
    private fun rawDistance(): Distance {
        val original = canRange.distance.value
        return original - canRangeOffset - SIDE_PLATE_THICKNESS
    }

    override fun periodic() {
        Logger.recordOutput("intake/coral_detected", hasBranchCoral)
        Logger.recordOutput("intake/coral_position", rawDistance())
        Logger.recordOutput("intake/coral_location", coralLocation)
        Logger.recordOutput("intake/supply_current", supplyCurrent)
        Logger.recordOutput("intake/torque_current", leader.torqueCurrent.value)
    }

    val intakeCoral by command {
        startEnd({ leader.set(0.3) }, { leader.set(0.0) }).withName("Intake Coral")
    }

    val outtakeCoral by command {
        startEnd({ leader.set(-0.7) }, { leader.set(0.0) }).withName("Outtake Coral")
    }

    val intakeAlgae by command {
        startEnd({ leader.set(0.3) }, { leader.set(0.0) }).withName("Intake Algae")
    }

    val outtakeAlgae by command {
        startEnd({ leader.set(-1.0) }, { leader.set(0.0) }).withName("Outtake Algae")
    }

    val scoreCoral by command {
        Commands.idle() // TODO score coral
    }

    // TODO find optimal intake and hold speeds experimentally
    fun intakeCoralThenHold(): Command =
        PrintCommand("Running HIGH until coral is detected.")
            .alongWith(runOnce({ leader.set(0.7) }))
            .andThen(Commands.idle())
            .until { hasGamePieceByCurrent(CORAL_CURRENT_THRESHOLD) }
            .andThen({ leader.set(0.15) })
}
