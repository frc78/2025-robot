package frc.robot.subsystems

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.amps
import frc.robot.lib.degrees
import frc.robot.lib.volts

object Wrist : SubsystemBase("Wrist") {
    val leader =
        TalonFX(13, "*").apply {
            configurator.apply(MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
        }
    var lowerLimit = 0.degrees
    var upperLimit = 0.degrees

    fun goTo(state: RobotState): Command =
        PrintCommand("Wrist going to $state - ${state.wristAngle}")
            .alongWith(runOnce { angle = state.wristAngle })

    var angle = 0.degrees
    val voltage = VoltageOut(0.0)

    fun manualUp(): Command {
        return startEnd(
            { leader.setControl(voltage.withOutput(2.0.volts)) },
            { leader.setControl(voltage.withOutput(0.0.volts)) },
        )
    }

    fun manualDown(): Command {
        return startEnd(
            { leader.setControl(voltage.withOutput(-2.0.volts)) },
            { leader.setControl(voltage.withOutput(0.0.volts)) },
        )
    }

    fun zeroRoutines(): Command {
        return SequentialCommandGroup(
            manualUp()
                .until({ leader.torqueCurrent.value > 10.0.amps })
                .andThen({ upperLimit = leader.position.value }),
            manualDown()
                .until({ leader.torqueCurrent.value > 10.0.amps })
                .andThen({ lowerLimit = leader.position.value }),
        )
    }

    fun resetPosition(position: Angle) {
        leader.setPosition(position)
    }
}
