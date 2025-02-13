package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DifferentialFollower
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.inches
import frc.robot.lib.toAngle
import frc.robot.lib.volts

object Elevator : SubsystemBase("Elevator") {
    val motionMagic = MotionMagicVoltage(0.0)
    val voltage = VoltageOut(0.0)

    fun goTo(state: RobotState): Command =
        PrintCommand("Elevator going to $state - ${state.elevatorHeight}")
            .alongWith(
                runOnce {
                    height = state.elevatorHeight
                    leader.setControl(
                        motionMagic.withPosition(state.elevatorHeight.toAngle(DRUM_RADIUS))
                    )
                }
            )

    fun manualUp() : Command {
        return startEnd({ leader.setControl(voltage.withOutput(2.0.volts)) }, { leader.setControl(voltage.withOutput(0.0.volts)) })
    }

    fun manualDown() : Command {
        return startEnd({ leader.setControl(voltage.withOutput(-2.0.volts)) }, { leader.setControl(voltage.withOutput(0.0.volts)) })
    }

    var height = 0.inches

    // Constants for the feedforward calculation
    private const val K_S = 0.070936
    private const val K_V = 0.79005
    private const val K_A = 0.086892
    private const val K_G = 0.088056

    // PID gains
    private const val K_P = 4.572
    private const val K_I = 0.0
    private const val K_D = 0.0

    private const val LEADER_MOTOR_ID = 11
    private const val FOLLOWER_MOTOR_ID = 12

    private const val GEAR_RATIO = 5.0
    private val DRUM_RADIUS = (1.75.inches + .25.inches) / 2.0

    private val leader =
        TalonFX(LEADER_MOTOR_ID, "*").apply {
            val leaderMotorConfiguration =
                TalonFXConfiguration().apply {
                    Feedback.SensorToMechanismRatio = GEAR_RATIO
                    SoftwareLimitSwitch.ForwardSoftLimitEnable = true
                    // Do not allow the motor to move upwards until after zeroing
                    SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0
                    // Allow the motor to move downwards until the limit switch is pressed
                    SoftwareLimitSwitch.ReverseSoftLimitEnable = false
                    SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0

                    MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

                    Slot0.kS = K_S
                    Slot0.kV = K_V
                    Slot0.kA = K_A
                    Slot0.kG = K_G
                    Slot0.kP = K_P
                    Slot0.kI = K_I
                    Slot0.kD = K_D
                    Slot0.GravityType = GravityTypeValue.Elevator_Static
                    Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign

                    MotionMagic.MotionMagicAcceleration = 80.0
                    MotionMagic.MotionMagicCruiseVelocity = 15.0
                }
            configurator.apply(leaderMotorConfiguration)
            position.setUpdateFrequency(100.0)
            velocity.setUpdateFrequency(100.0)
            motorVoltage.setUpdateFrequency(100.0)
            closedLoopError.setUpdateFrequency(100.0)
        }

    val follower =
        TalonFX(FOLLOWER_MOTOR_ID, "*").apply {
            setControl(Follower(LEADER_MOTOR_ID, true))
        }
}
