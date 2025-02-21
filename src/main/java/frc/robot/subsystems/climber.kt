package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.volts


object climber : SubsystemBase("climber") {

    private val Motor =
    TalonFX(16, "*").apply {
        val config =
            TalonFXConfiguration().apply {
                MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
                MotorOutput.NeutralMode = NeutralModeValue.Brake

            }
    }

    val voltageOut = VoltageOut(0.0)

    fun climb(): Command {
        return startEnd(
            { Motor.setControl(voltageOut.withOutput(2.0.volts)) },
            { Motor.setControl(voltageOut.withOutput(0.0.volts)) },
        )
    }




