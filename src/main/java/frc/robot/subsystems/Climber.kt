package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.command
import frc.robot.lib.rotations
import org.littletonrobotics.junction.Logger

object Climber : SubsystemBase("climber") {

    private val rotationsPerInch = 12.0.rotations
    private const val EXTENDED_INCHES = 6.0
    private val extendedPosition = rotationsPerInch * EXTENDED_INCHES

    private val positionVoltage = PositionVoltage(0.0)

    val isExtended: Boolean
        get() = leader.position.value > extendedPosition / 2.0

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

    private var setpoint = 0.rotations

    override fun periodic() {
        Logger.recordOutput("climber/position", leader.position.value)
        leader.setControl(
            positionVoltage
                .withPosition(setpoint)
                .withLimitForwardMotion(Pivot.angle > Pivot.EXTEND_FOOT_THRESHOLD)
        )
    }

    val retract by command { runOnce { setpoint = 0.rotations }.withName("Retract foot") }

    init {
        SmartDashboard.putData(retract)
    }

    val extend by command { runOnce { setpoint = extendedPosition }.withName("Extend Foot") }
}
