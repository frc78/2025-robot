package frc.robot.subsystems

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.UpdateModeValue
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.*
import frc.robot.lib.bindings.ReefscapeController
import frc.robot.subsystems.Intake.IntakeState.*
import java.util.function.BooleanSupplier
import org.littletonrobotics.junction.Logger

object Intake : SubsystemBase("intake") {

    enum class IntakeState(val control: ControlRequest) {
        // Separate intake vs holding states to allow going home if game piece isn't acquired
        INTAKING_CORAL(TorqueCurrentFOC(20.amps)),
        HOLDING_CORAL(TorqueCurrentFOC(20.amps)),
        INTAKING_ALGAE(TorqueCurrentFOC((-40).amps)),
        HOLDING_ALGAE(TorqueCurrentFOC((-40).amps)),
        EJECTING_CORAL(DutyCycleOut(-.5)),
        NETTING_ALGAE(DutyCycleOut(0.2)),
        PROCESSING_ALGAE(DutyCycleOut(0.1)),
        HOME(DutyCycleOut(0.0));

        fun transition(to: IntakeState, condition: BooleanSupplier) {
            Trigger { currentState == this }.and(condition).onTrue(runOnce { currentState = to })
        }
    }

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

    private val CAN_RANGE_OFFSET = 19.0.centimeters

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

    private var currentState = HOME

    private val intakeAlgae =
        ReefscapeController.floorAlgae()
            .or(ReefscapeController.lowAlgae())
            .or(ReefscapeController.highAlgae())

    init {
        defaultCommand = run { leader.setControl(currentState.control) }
        HOME.apply {
            transition(INTAKING_CORAL, ReefscapeController.coral())
            transition(INTAKING_ALGAE, intakeAlgae)
        }
        INTAKING_CORAL.apply {
            transition(HOME, ReefscapeController.home())
            transition(HOLDING_CORAL, { holdingCoral })
            transition(INTAKING_ALGAE, intakeAlgae)
        }
        INTAKING_ALGAE.apply {
            transition(HOME, ReefscapeController.home())
            transition(HOLDING_ALGAE, { holdingAlgae })
            transition(INTAKING_CORAL, ReefscapeController.coral())
        }
        HOLDING_CORAL.apply { transition(EJECTING_CORAL, ReefscapeController.score()) }
        HOLDING_ALGAE.apply {
            transition(
                NETTING_ALGAE,
                ReefscapeController.score().and {
                    true
                }, /* replace with check for superstructure state */
            )
            transition(
                PROCESSING_ALGAE,
                ReefscapeController.score().and {
                    false /* replace with check for superstructure state */
                },
            )
        }
        EJECTING_CORAL.apply {
            transition(HOME, ReefscapeController.home())
            transition(INTAKING_CORAL, ReefscapeController.coral())
            transition(
                INTAKING_ALGAE,
                ReefscapeController.lowAlgae()
                    .or(ReefscapeController.highAlgae())
                    .or(ReefscapeController.floorAlgae()),
            )
        }
        NETTING_ALGAE.apply {
            transition(HOME, ReefscapeController.home())
            transition(INTAKING_CORAL, ReefscapeController.coral())
            transition(
                INTAKING_ALGAE,
                ReefscapeController.lowAlgae()
                    .or(ReefscapeController.highAlgae())
                    .or(ReefscapeController.floorAlgae()),
            )
        }
        PROCESSING_ALGAE.apply {
            transition(INTAKING_CORAL, ReefscapeController.coral())
            transition(INTAKING_ALGAE, intakeAlgae)
            transition(HOME, ReefscapeController.home())
        }
    }

    /**
     * Returns true if a coral is detected in the path of the CANrange. Only corals that are
     * oriented vertically, and thus able to be scored on one of the reef branches, will be detected
     */
    private val hasBranchCoral: Boolean
        get() = canRange.isDetected.value

    val holdingCoral
        get() = leader.torqueCurrent.valueAsDouble > 15 && leader.velocity.valueAsDouble < 5

    val holdingAlgae
        get() = leader.torqueCurrent.valueAsDouble < -15 && leader.velocity.valueAsDouble > -5

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
            return canRange.distance.value - CAN_RANGE_OFFSET
        }

    override fun periodic() {
        Logger.recordOutput("intake/state", currentState.name)
        Logger.recordOutput("intake/coral_detected", hasBranchCoral)
        Logger.recordOutput("intake/coral_position", canRange.distance.value)
        Logger.recordOutput("intake/coral_location", coralLocation)
        Logger.recordOutput("intake/torque_current", leader.torqueCurrent.value)
        Logger.recordOutput("intake/has_algae", holdingAlgae)
        Logger.recordOutput("intake/speed", leader.get())
    }

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
