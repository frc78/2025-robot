package frc.robot.subsystems

import com.ctre.phoenix6.configs.CANrangeConfiguration
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
import frc.robot.lib.meters
import org.littletonrobotics.junction.Logger

object Intake : Subsystem {
    init {
        defaultCommand = Commands.idle(this)
    }

    private val canRange: CANrange =
        CANrange(0, "*").apply {
            configurator.apply(
                CANrangeConfiguration().apply {
                    FovParams.FOVRangeX = 6.0
                    // High FOV along length of coral
                    FovParams.FOVRangeY = 27.0
                    ProximityParams.ProximityThreshold = 40.centimeters.meters
                }
            )
        }

    private val canRangeOffsetEntry =
        NetworkTableInstance.getDefault().getTable("intake").getEntry("canRangeOffset").also {
            it.setDouble(19.0)
        }

    private val canRangeOffset
        get() = canRangeOffsetEntry.getDouble(19.0).centimeters

    private val CORAL_CURRENT_THRESHOLD =
        25.amps // Current spike threshold for detecting when we have a coral

    private val ALGAE_CURRENT_THRESHOLD =
        (-30).amps // Current spike threshold for detecting when we have an algae
    private var algaeSpikeDetected = false
    private var algaeSpikeResolved = false

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

    val torqueCurrent: Current
        get() = leader.torqueCurrent.value

    /**
     * Return true if the intake motor is experiencing current draw greater than the given
     * threshold.
     */
    fun hasCoralByCurrent(): Boolean {
        // return true if current is spiked and coral is detected by CANRange
        return (leader.torqueCurrent.value >= CORAL_CURRENT_THRESHOLD) && hasBranchCoral
    }

    fun hasAlgaeByCurrent(): Boolean {
        // NOTE: Algae intake current is negative
        val thresholdMet: Boolean = leader.torqueCurrent.value <= ALGAE_CURRENT_THRESHOLD
        if (thresholdMet) {
            if (algaeSpikeDetected) {
                if (algaeSpikeResolved) {
                    // if threshold is met, and initial current spike from running motor is already
                    // accounted for,
                    // we have the algae so return true and reset the spike detection booleans
                    algaeSpikeDetected = false
                    algaeSpikeResolved = false
                    return true
                }
            } else {
                // threshold met and initial spike not detected yet, so mark it as such
                algaeSpikeDetected = true
            }
        } else if (algaeSpikeDetected) {
            // if spike was detected but current is back down, is now resolved
            algaeSpikeResolved = true
        }
        return false
    }

    // Returns the distance from the center of the intake to the center of the coral.
    // TODO check this formula with the new intake
    val coralLocation: Distance
        get() {
            if (!hasBranchCoral) {
                return 0.0.centimeters
            }

            return -(canRange.distance.value - canRangeOffset)
        }

    override fun periodic() {
        Logger.recordOutput("intake/coral_detected", hasBranchCoral)
        Logger.recordOutput("intake/coral_position", canRange.distance.value)
        Logger.recordOutput("intake/coral_location", coralLocation)
        Logger.recordOutput("intake/supply_current", supplyCurrent)
        Logger.recordOutput("intake/torque_current", leader.torqueCurrent.value)
    }

    val intakeCoral by command {
        startEnd({ leader.set(0.6) }, { leader.set(0.0) }).withName("Intake Coral")
    }

    val outtakeCoral by command {
        startEnd({ leader.set(-1.0) }, { leader.set(0.0) }).withName("Outtake Coral")
    }

    val intakeAlgae by command {
        startEnd({ leader.set(-1.0) }, { leader.set(0.0) }).withName("Intake Algae")
    }

    val outtakeAlgae by command {
        startEnd({ leader.set(1.0) }, { leader.set(0.0) }).withName("Outtake Algae")
    }

    val scoreCoral by command {
        Commands.idle() // TODO score coral
    }

    // TODO find optimal intake and hold speeds experimentally
    fun intakeCoralThenHold(): Command =
        PrintCommand("Intake coral then hold")
            .alongWith(runOnce { leader.set(0.6) })
            .andThen(Commands.idle())
            .until { hasCoralByCurrent() }
            .andThen({ leader.set(0.08) })

    fun intakeAlgaeThenHold(): Command =
        PrintCommand("Intake algae then hold")
            .alongWith(runOnce { leader.set(-1.0) })
            .andThen(Commands.idle())
            .until { hasAlgaeByCurrent() }
            .andThen({ leader.set(-0.5) })
}
