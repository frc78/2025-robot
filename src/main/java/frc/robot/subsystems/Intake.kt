package frc.robot.subsystems

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.UpdateModeValue
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.IS_COMP
import frc.robot.lib.amps
import frc.robot.lib.centimeters
import frc.robot.lib.command
import frc.robot.lib.kilogramSquareMeters
import frc.robot.lib.meters
import frc.robot.lib.poundSquareInches
import frc.robot.lib.seconds
import org.littletonrobotics.junction.Logger

object Intake : SubsystemBase("intake") {

    private val canRange: CANrange =
        CANrange(0, "*").apply {
            configurator.apply(
                CANrangeConfiguration().apply {
                    FovParams.FOVRangeX = 6.0
                    // High FOV along length of coral
                    FovParams.FOVRangeY = 27.0
                    ProximityParams.ProximityThreshold = 40.centimeters.meters
                    ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz
                }
            )
        }

    private val canRangeOffsetEntry =
        NetworkTableInstance.getDefault().getTable("intake").getEntry("canRangeOffset").also {
            it.setDouble(19.0)
        }

    private val canRangeOffset
        get() = canRangeOffsetEntry.getDouble(19.0).centimeters

    /* Current spike threshold for detecting when we have a coral*/
    private val CORAL_CURRENT_THRESHOLD = 15.amps

    /* Current spike threshold for detecting when we have an algae*/
    private val ALGAE_CURRENT_THRESHOLD = (-25).amps

    private val ALPHA_BOT_MOTOR_OUTPUT_CONFIG =
        MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
    private val COMP_BOT_MOTOR_OUTPUT_CONFIG =
        MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
    private val leader = // used to be the algae motor, now is the only motor on new rev of intake
        TalonFX(15, "*").apply {
            configurator.apply(
                TalonFXConfiguration().apply {
                    CurrentLimits.StatorCurrentLimit = 40.0
                    CurrentLimits.SupplyCurrentLimit = 20.0
                    MotorOutput =
                        if (IS_COMP) COMP_BOT_MOTOR_OUTPUT_CONFIG else ALPHA_BOT_MOTOR_OUTPUT_CONFIG
                    OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5
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

    val torqueCurrent: Current
        get() = leader.torqueCurrent.value

    private val torqueCurrentFilter = LinearFilter.singlePoleIIR(0.1, 0.02)

    /**
     * Return true if the intake motor is experiencing current draw greater than the given
     * threshold.
     */
    fun hasCoralByCurrent(): Boolean {
        return (leader.torqueCurrent.value >= CORAL_CURRENT_THRESHOLD) &&
            coralDetectedDebounce.calculate(hasBranchCoral)
    }

    val hasAlgaeByCurrent
        get() = torqueCurrentFilter.lastValue() <= ALGAE_CURRENT_THRESHOLD.amps

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
            return canRange.distance.value - canRangeOffset
        }

    override fun periodic() {
        Logger.recordOutput("intake/coral_detected", hasBranchCoral)
        Logger.recordOutput("intake/coral_location", coralLocation)
        Logger.recordOutput("intake/has_algae", hasAlgaeByCurrent)
        Logger.recordOutput("intake/speed", leader.get())
        torqueCurrentFilter.calculate(torqueCurrent.amps)
    }

    val manualIntake by command { startEnd({ leader.set(1.0) }, { leader.set(0.0) }) }

    val manualOuttake by command { startEnd({ leader.set(-1.0) }, { leader.set(0.0) }) }

    val outtakeCoral by command {
        startEnd({ leader.set(-0.5) }, { leader.set(0.0) }).withName("outtakeCoral")
    }

    private fun outtakeAlgae(speed: () -> Double) = startEnd({ leader.set(speed()) }, { leader.set(0.0) }).withName("outtakeAlgae")

    /** Outtake and then stop after delay */
    val scoreCoral by command { outtakeCoral.withTimeout(0.2.seconds) }
    val scoreAlgae by command { outtakeAlgae { if (Elevator.position.meters <= 0.2) 0.2 else 1.0 }.withTimeout(0.5.seconds) }

    // Debouncer to allow coral to be fully in before we use the distance measurement
    private val coralDetectedDebounce = Debouncer(0.1, Debouncer.DebounceType.kRising)

    fun intakeCoralThenHold(): Command =
        startEnd({ leader.set(0.7) }, { leader.set(0.035) })
            .until { hasCoralByCurrent() }
            .withName("Intake coral then hold")

    val overIntakeCoralThenHold by command {
        Commands.sequence(
            intakeCoral,
            Commands.waitUntil { hasCoralByCurrent() },
            Commands.waitSeconds(0.2),
            holdCoral,
        )
    }
    val intakeCoral by command { runOnce { leader.set(0.7) } }
    val holdCoral by command { runOnce { leader.set(0.1) } }

    fun intakeAlgaeThenHold(): Command =
        startEnd({ leader.set(-1.0) }, { leader.set(-0.6) })
            .until { hasAlgaeByCurrent }
            .withName("Intake algae then hold")

    private val sim =
        FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                8.28.poundSquareInches.kilogramSquareMeters,
                36.0 / 12.0,
            ),
            DCMotor.getKrakenX60Foc(1),
            0.0,
        )

    private val simState by lazy { leader.simState }

    override fun simulationPeriodic() {
        sim.inputVoltage = simState.motorVoltage
        sim.update(0.02)
        simState.setRotorVelocity(sim.angularVelocity)
    }
}
