package frc.robot.subsystems

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.UpdateModeValue
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.*
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
    /** Current spike threshold for detecting when we have a coral */
    private val CORAL_CURRENT_THRESHOLD = 15.amps

    private val IntakeCoralControl = TorqueCurrentFOC(15.amps)
    private val IntakeAlgaeControl = TorqueCurrentFOC((-15).amps)

    /** Current spike threshold for detecting when we have an algae */
    private val ALGAE_CURRENT_THRESHOLD = (-25).amps

    private val leader = // used to be the algae motor, now is the only motor on new rev of intake
        TalonFX(15, "*").apply {
            configurator.apply(
                TalonFXConfiguration().apply {
                    CurrentLimits.StatorCurrentLimit = 40.0
                    CurrentLimits.SupplyCurrentLimit = 20.0
                    MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
                }
            )
        }

    init {
        leader.set(0.0)
    }

    private val coralIntakeDebounce = Debouncer(0.2, Debouncer.DebounceType.kRising)
    /**
     * Return true if the intake motor is experiencing current draw greater than the given
     * threshold.
     */
    val holdingCoral
        get() =
            coralIntakeDebounce.calculate(
                leader.torqueCurrent.value >= CORAL_CURRENT_THRESHOLD &&
                    leader.velocity.valueAsDouble <= 5
            )

    private val algaeIntakeDebounce = Debouncer(0.2, Debouncer.DebounceType.kRising)

    val holdingAlgae
        get() =
            algaeIntakeDebounce.calculate(
                leader.torqueCurrent.value <= ALGAE_CURRENT_THRESHOLD &&
                    leader.velocity.valueAsDouble >= -5
            )

    /** Returns the distance from the center of the intake to the center of the coral. */
    val coralLocation: Distance
        get() {
            if (!canRange.isDetected.value) {
                return 0.0.centimeters
            }

            /* The CANrange is mounted looking from +X to -X, so the coral position increases as it
            goes toward -X The offset is the value that puts the coral at 0 when it is in the middle of the intake.
            Because the axis are inverse from each other (-X for the CANrange goes in the same direction as +X for
            the robot), the adjusted values must be negated in order to sync the two coordinate systems */
            return canRange.distance.value - 19.0.centimeters
        }

    override fun periodic() {
        Logger.recordOutput("intake/coral_location", coralLocation)
        Logger.recordOutput("intake/holding_algae", holdingAlgae)
        Logger.recordOutput("intake/holding_coral", holdingAlgae)
    }

    val outtakeCoral by command {
        startEnd({ leader.set(-0.5) }, { leader.set(0.0) }).withName("outtakeCoral")
    }

    private fun outtakeAlgae(speed: () -> Double) =
        startEnd({ leader.set(speed()) }, { leader.set(0.0) }).withName("outtakeAlgae")

    /** Outtake and then stop after delay */
    val scoreCoral by command { outtakeCoral.withTimeout(0.2.seconds) }
    val scoreAlgae by command {
        outtakeAlgae { if (Elevator.position.meters <= 0.2) 0.2 else 1.0 }.withTimeout(0.7.seconds)
    }

    val intakeCoral by command { runOnce { leader.setControl(IntakeCoralControl) } }
    val intakeAlgae by command { runOnce { leader.setControl(IntakeAlgaeControl) } }

    private val sim =
        FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                .25.poundSquareInches.kilogramSquareMeters,
                36.0 / 12.0,
            ),
            DCMotor.getKrakenX60Foc(1),
            0.0,
        )

    private val simState by lazy { leader.simState }

    override fun simulationPeriodic() {
        sim.inputVoltage = simState.motorVoltage
        sim.update(0.02)
        simState.setRotorVelocity(sim.angularVelocity * 36.0 / 12.0)
    }
}
