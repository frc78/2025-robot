package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.rotations
import frc.robot.subsystems.SuperStructure.SuperStructureState.FullyClimbed
import org.littletonrobotics.junction.Logger

object Climber : SubsystemBase("climber") {

    enum class ClimberState(val control: ControlRequest) {
        Retracted(PositionVoltage(0.0.rotations)),
        Extended(PositionVoltage(extendedPosition))
    }
    private val rotationsPerInch = 12.0.rotations
    private const val EXTENDED_INCHES = 6.0
    private val extendedPosition = rotationsPerInch * EXTENDED_INCHES


    init {
        defaultCommand = run { leader.setControl(currentState.control)}
        bind(ClimberState.Retracted).and{SuperStructure.currentState == FullyClimbed }.onTrue(runOnce { currentState = ClimberState.Extended })
    }

    private var currentState = ClimberState.Retracted
    fun bind(state: ClimberState) = Trigger{ currentState == state }

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

    override fun periodic() {
        Logger.recordOutput("climber/position", leader.position.value)
    }

}
