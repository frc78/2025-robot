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
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.robot.lib.*
import frc.robot.lib.bindings.ReefscapeController
import frc.robot.subsystems.Intake.IntakeState.*
import org.littletonrobotics.junction.Logger

object Intake {

    enum class IntakeState(val control: ControlRequest) {
        Idle(DutyCycleOut(0.0)),
        // Separate intake vs holding states to allow going home if game piece isn't acquired
        IntakeCoral(TorqueCurrentFOC(20.amps)),
        HoldCoral(TorqueCurrentFOC(20.amps)),
        EjectCoral(DutyCycleOut(-.5)),
        IntakeAlgae(TorqueCurrentFOC((-40).amps)),
        HoldAlgae(TorqueCurrentFOC((-40).amps)),
        NetAlgae(DutyCycleOut(0.2)),
        ProcessAlgae(DutyCycleOut(0.1)),
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

    private val stateTimer = Timer()
    var currentState = Idle
        set(value) {
            if (field != value) {
                stateTimer.restart()
            }
            field = value
        }

    fun stateMachine() {
        leader.setControl(currentState.control)
        val intakeAlgae =
            ReefscapeController.floorAlgae()
                .or(ReefscapeController.lowAlgae())
                .or(ReefscapeController.highAlgae())
        when (currentState) {
            Idle -> {
                if (ReefscapeController.coral()) {
                    currentState = IntakeCoral
                } else if (intakeAlgae) {
                    currentState = IntakeAlgae
                }
            }

            IntakeCoral -> {
                if (ReefscapeController.home()) {
                    currentState = Idle
                } else if (holdingCoral) {
                    currentState = HoldCoral
                }
            }

            HoldCoral -> {
                if (ReefscapeController.score()) {
                    currentState = EjectCoral
                }
            }

            EjectCoral -> {
                if (ReefscapeController.home()) {
                    currentState = Idle
                } else if (ReefscapeController.coral()) {
                    currentState = IntakeCoral
                }
            }

            IntakeAlgae -> {
                if (ReefscapeController.home()) {
                    currentState = Idle
                } else if (holdingAlgae) {
                    currentState = HoldAlgae
                }
            }

            HoldAlgae -> {
                if (ReefscapeController.score()) {
                    currentState = ProcessAlgae
                }
            }

            NetAlgae -> {
                if (ReefscapeController.home()) {
                    currentState = Idle
                } else if (ReefscapeController.coral()) {
                    currentState = IntakeCoral
                } else if (intakeAlgae) {
                    currentState = IntakeAlgae
                }
            }

            ProcessAlgae -> {
                if (ReefscapeController.home()) {
                    currentState = Idle
                } else if (ReefscapeController.coral()) {
                    currentState = IntakeCoral
                } else if (intakeAlgae) {
                    currentState = IntakeAlgae
                }
            }
        }
    }

    /**
     * Returns true if a coral is detected in the path of the CANrange. Only corals that are
     * oriented vertically, and thus able to be scored on one of the reef branches, will be detected
     */
    private val hasBranchCoral: Boolean
        get() = canRange.isDetected.value

    private val holdCoralDebounce = Debouncer(0.5, Debouncer.DebounceType.kRising)
    val holdingCoral
        get() =
            holdCoralDebounce.calculate(
                leader.torqueCurrent.valueAsDouble > 15 && leader.velocity.valueAsDouble < 5
            )

    private val holdAlgaeDebounce = Debouncer(0.5, Debouncer.DebounceType.kRising)
    val holdingAlgae
        get() =
            holdAlgaeDebounce.calculate(
                leader.torqueCurrent.valueAsDouble < -15 && leader.velocity.valueAsDouble > -5
            )

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

    fun periodic() {
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
                .263.poundSquareInches.kilogramSquareMeters,
                36.0 / 12.0,
            ),
            DCMotor.getKrakenX60Foc(1),
            0.0,
        )

    private val simState by lazy { leader.simState }

    fun simulationPeriodic() {
        sim.inputVoltage = simState.motorVoltage
        sim.update(0.02)
        if (
            currentState == IntakeCoral ||
                currentState == HoldCoral ||
                currentState == IntakeState.IntakeAlgae ||
                currentState == IntakeState.HoldAlgae
        ) {
            simState.setRotorVelocity(0.0)
        } else {
            simState.setRotorVelocity(sim.angularVelocity * 36.0 / 12.0)
        }
    }
}
