package frc.robot.subsystems

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.amps
import frc.robot.lib.centimeters
import frc.robot.lib.command
import frc.robot.lib.meters
import org.littletonrobotics.junction.Logger

object Intake : SubsystemBase("Intake") {
    init {
        SmartDashboard.putData(this)
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

    /** Current spike threshold for detecting when we have a coral */
    private val CORAL_CURRENT_THRESHOLD = 25.amps

    /** Current spike threshold for detecting when we have an algae */
    private val ALGAE_CURRENT_THRESHOLD = (-30).amps

    private var algaeSpikeDetected = false
    private var algaeSpikeResolved = false

    // used to be the algae motor, now is the only motor on new rev of intake
    private val leader =
        TalonFX(15, "*").apply {
            configurator.apply(
                TalonFXConfiguration().apply {
                    CurrentLimits.StatorCurrentLimit = 40.0
                    CurrentLimits.SupplyCurrentLimit = 20.0
                    MotorOutput.Inverted = InvertedValue.Clockwise_Positive
                }
            )
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
    val hasCoralByCurrent
        get() = (leader.torqueCurrent.value >= CORAL_CURRENT_THRESHOLD) && hasBranchCoral

    val hasAlgaeByCurrent: Boolean
        get() {
            // NOTE: Algae intake current is negative
            val thresholdMet = torqueCurrent <= ALGAE_CURRENT_THRESHOLD
            if (thresholdMet) {
                if (algaeSpikeDetected) {
                    if (algaeSpikeResolved) {
                        // if threshold is met, and initial current spike from running motor is
                        // already
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

    /** Returns the distance from the center of the intake to the center of the coral. */
    val coralLocation: Distance
        get() {
            if (!hasBranchCoral) {
                return 0.0.centimeters
            }

            /* The CANrange is mounted looking from +X to -X, so the coral position increases as it
            goes toward -X The offset is the value that puts the coral at 0 when it is in the middle of the intake.
            Because the axis are inverse from each other (-X for the CANrange goes in the same direction as +X for
            the robot), the adjusted values must be negated in order to sync the two coordinate systems */
            return -(canRange.distance.value - canRangeOffset)
        }

    override fun periodic() {
        Logger.recordOutput("intake/coral_detected", hasBranchCoral)
        Logger.recordOutput("intake/canrange_distance", canRange.distance.value)
        Logger.recordOutput("intake/coral_location", coralLocation)
        Logger.recordOutput("intake/supply_current", supplyCurrent)
        Logger.recordOutput("intake/torque_current", torqueCurrent)
        Logger.recordOutput("intake/has_coral_by_current", hasCoralByCurrent)
        Logger.recordOutput("intake/has_algae_by_current", hasAlgaeByCurrent)
    }

    val outtakeCoral by command {
        startEnd({ leader.set(-1.0) }, { leader.set(0.0) }).withName("Outtake Coral")
    }

    val outtakeAlgae by command {
        startEnd({ leader.set(1.0) }, { leader.set(0.0) }).withName("Outtake Algae")
    }

    /** Outtake coral until the coral is no longer detected, plus .3 seconds */
    val scoreCoral by command {
        Commands.waitUntil { !hasBranchCoral }
            .andThen(Commands.waitSeconds(0.3))
            .deadlineFor(outtakeCoral)
            .withName("Score Coral")
    }

    /** Outtake coral until the coral is no longer detected, plus .3 seconds */
    val scoreAlgae by command { outtakeAlgae.withTimeout(0.5).withName("Score Algae") }

    /** Run coral intake until it is detected, then hold the coral */
    val intakeCoralThenHold by command {
        PrintCommand("Intake coral then hold")
            .alongWith(runOnce { leader.set(0.6) })
            .andThen(Commands.idle().until { hasCoralByCurrent })
            .finallyDo { interrupted -> if (interrupted) leader.set(0.0) else leader.set(0.08) }
            .withName("Intake Algae Then Hold")
    }

    /** Run algae intake until it is detected, then hold the algae */
    val intakeAlgaeThenHold by command {
        PrintCommand("Intake algae then hold")
            .alongWith(runOnce { leader.set(-1.0) })
            .andThen(Commands.idle().until { hasAlgaeByCurrent })
            .finallyDo { interrupted -> if (interrupted) leader.set(0.0) else leader.set(-0.5) }
            .withName("Intake Algae Then Hold")
    }
}
