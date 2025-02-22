package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.command

object Climber : SubsystemBase("Climber") {
    private val leader =
        TalonFX(16, "*").apply {
            val config =
                TalonFXConfiguration().apply {
                    MotorOutput.Inverted = InvertedValue.Clockwise_Positive
                    MotorOutput.NeutralMode = NeutralModeValue.Brake
                }

            configurator.apply(config)
        }

    private val voltageOut = VoltageOut(0.0)

    val runRoller by command {
        startEnd(
            //            { leader.setControl(voltageOut.withOutput(2.0.volts)) },
            //            { leader.setControl(voltageOut.withOutput(0.0.volts)) },
            { leader.set(0.75) },
            { leader.set(0.0) },
        )
    }

    val reverseRoller by command {
        startEnd(
            //            { leader.setControl(voltageOut.withOutput((-2.0).volts)) },
            //            { leader.setControl(voltageOut.withOutput(0.0.volts)) },
            { leader.set(-0.75) },
            { leader.set(0.0) },
        )
    }
}
