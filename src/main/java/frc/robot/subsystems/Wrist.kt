package frc.robot.subsystems
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import frc.robot.lib.degrees
import frc.robot.lib.volts


object Wrist : SubsystemBase("Wrist") {
    val leader = TalonFX(-1, "*")

    fun goTo(state: RobotState): Command =
        PrintCommand("Wrist going to $state - ${state.wristAngle}")
            .alongWith(runOnce { angle = state.wristAngle })

    var angle = 0.degrees
    val voltage = VoltageOut(0.0)

    fun manualUp() : Command {
        return startEnd({ leader.setControl(voltage.withOutput(2.0.volts)) }, { leader.setControl(voltage.withOutput(0.0.volts)) })
    }

    fun manualDown() : Command {
        return startEnd({ leader.setControl(voltage.withOutput(-2.0.volts)) }, { leader.setControl(voltage.withOutput(0.0.volts)) })
    }
}
