package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.command
import frc.robot.lib.rotations
import org.littletonrobotics.junction.Logger

object Climber : SubsystemBase("Climber") {

    private val rotationsPerInch = 12.0.rotations
    private const val EXTENDED_INCHES = 6.0
    private val extendedPosition = rotationsPerInch * EXTENDED_INCHES

    private val positionVoltage = PositionVoltage(0.0)

    val isExtended: Boolean
        get() = leader.position.value > extendedPosition / 2.0

    private val leader =
        TalonFX(14, "*").apply {
            configurator.apply(
                TalonFXConfiguration().apply {
                    CurrentLimits.StatorCurrentLimit = 40.0
                    CurrentLimits.SupplyCurrentLimit = 20.0
                    MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
                    Feedback.SensorToMechanismRatio = 20.0 / 12.0
                    Slot0.kP = 10.0
                }
            )
        }

    init {
        defaultCommand = Commands.idle(this).withName("Climber idle")
        leader.set(0.0)
    }

    override fun periodic() {
        Logger.recordOutput("climber/position", leader.position.value)
    }

    val retract by command {
        runOnce { leader.setControl(positionVoltage.withPosition(0.0)) }.withName("Retract Foot")
    }

    val extend by command {
        runOnce { leader.setControl(positionVoltage.withPosition(extendedPosition)) }
            .withName("Extend Foot")
    }

    init {
        SmartDashboard.putData(retract)
        SmartDashboard.putData(extend)
    }
}
