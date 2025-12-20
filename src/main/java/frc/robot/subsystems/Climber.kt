package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import frc.robot.lib.rotations
import frc.robot.subsystems.SuperStructure.SuperStructureState.FullyClimbed
import org.littletonrobotics.junction.Logger

object Climber {

    enum class ClimberState(val control: ControlRequest) {
        Retracted(PositionVoltage(0.0.rotations)),
        Extended(PositionVoltage(EXTENDED_ROTATIONS)),
    }

    // 12 TPI leadscrew
    private const val LEADSCREW_TPI = 12.0
    private const val EXTENDED_INCHES = 6.0
    private const val EXTENDED_ROTATIONS = LEADSCREW_TPI * EXTENDED_INCHES

    fun stateMachine() {
        leader.setControl(state.control)
        when (state) {
            ClimberState.Retracted -> {
                if (SuperStructure.state == FullyClimbed) {
                    state = ClimberState.Extended
                }
            }
            ClimberState.Extended -> {
                // No automatic transition out of extended
            }
        }
    }

    private var state = ClimberState.Retracted

    private val leader =
        TalonFX(16, "*").apply {
            configurator.apply(
                TalonFXConfiguration().apply {
                    CurrentLimits.StatorCurrentLimit = 80.0
                    CurrentLimits.SupplyCurrentLimit = 40.0
                    MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
                    Feedback.SensorToMechanismRatio = 20.0 / 12.0
                    Slot0.kP = 10.0
                }
            )
        }

    fun periodic() {
        Logger.recordOutput("climber/position", leader.position.value)
    }
}
