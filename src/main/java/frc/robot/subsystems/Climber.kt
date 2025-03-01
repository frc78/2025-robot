package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.command

object Climber : SubsystemBase("Climber") {

    private val leader =
        TalonFX(14, "*").apply {
            configurator.apply(
                TalonFXConfiguration().apply {
                    CurrentLimits.StatorCurrentLimit = 40.0
                    CurrentLimits.SupplyCurrentLimit = 20.0
                    MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
                }
            )
        }

    init {
        defaultCommand = Commands.idle(this)
        leader.set(0.0)
        SmartDashboard.putData(this)
    }

    override fun periodic() {}

    val manualExtend by command {
        startEnd({ leader.set(.5) }, { leader.set(0.0) }).withName("Extend Climber")
    }

    val manualRetract by command {
        startEnd({ leader.set(-.5) }, { leader.set(0.0) }).withName("Retract Climber")
    }
}
