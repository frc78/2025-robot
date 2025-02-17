package frc.robot.subsystems

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.*
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.lib.*
import frc.robot.lib.amps
import frc.robot.lib.degrees
import frc.robot.lib.volts

object Wrist : SubsystemBase("Wrist") {
    private var lowerLimit = 0.degrees
    private var upperLimit = 120.degrees
    private const val MOTOR_TO_WRIST = 104.1429

    private val leader =
        TalonFX(13, "*").apply {
            val config =
                TalonFXConfiguration().apply {
                    Feedback.SensorToMechanismRatio = MOTOR_TO_WRIST
                    MotorOutput.Inverted = InvertedValue.Clockwise_Positive
                    MotorOutput.NeutralMode = NeutralModeValue.Brake

                    SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(upperLimit)
                        .withReverseSoftLimitThreshold(lowerLimit)

                    Slot0.withKP(62.105)
                        .withKD(19.613)
                        .withKS(0.0)
                        .withKV(0.0)
                        .withKA(0.0)
                        .withKG(0.0)
                        .withGravityType(GravityTypeValue.Arm_Cosine)

                    MotionMagic.MotionMagicCruiseVelocity = .25
                    MotionMagic.MotionMagicAcceleration = 2.5
                }

            configurator.apply(config)
        }

    val motionMagic = MotionMagicVoltage(0.degrees)
    val positionVoltage = PositionVoltage(0.degrees)
    val voltageOut = VoltageOut(0.0)

    fun goTo(state: RobotState): Command =
        PrintCommand("Wrist going to $state - ${state.wristAngle}")
            .alongWith(
                runOnce {
                    leader.setControl(
                        positionVoltage.withPosition(state.wristAngle)
                    ) // Do we need to do .wristToMotor()
                }
            )

    val angle: Angle
        get() = leader.position.value

    fun manualUp(): Command {
        return startEnd(
            { leader.setControl(voltageOut.withOutput(2.0.volts)) },
            { leader.setControl(voltageOut.withOutput(0.0.volts)) },
        )
    }

    fun manualDown(): Command {
        return startEnd(
            { leader.setControl(voltageOut.withOutput((-2.0).volts)) },
            { leader.setControl(voltageOut.withOutput(0.0.volts)) },
        )
    }

    val resetPosition: Command = Commands.runOnce({ leader.setPosition(0.0) })

    fun zeroRoutines(): Command {
        return SequentialCommandGroup(
            manualDown()
                .until({ leader.torqueCurrent.value > 10.0.amps })
                .andThen({
                    leader.setPosition(0.0.degrees)
                    lowerLimit = leader.position.value + 5.degrees
                }),
            manualUp()
                .until({ leader.torqueCurrent.value > 10.0.amps })
                .andThen({ upperLimit = leader.position.value - 5.degrees }),
        )
    }

    private val sysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(
                .3.voltsPerSecond,
                3.volts,
                10.seconds,
                { SignalLogger.writeString("wrist_state", "$it") },
            ),
            SysIdRoutine.Mechanism(
                { leader.setControl(voltageOut.withOutput(it)) },
                null,
                this,
                "wrist",
            ),
        )

    private val sysId =
        Commands.sequence(
                runOnce { SignalLogger.start() },
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until {
                    leader.position.value >= upperLimit
                },
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until {
                    leader.position.value <= lowerLimit
                },
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until {
                    leader.position.value >= upperLimit
                },
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until {
                    leader.position.value <= lowerLimit
                },
                runOnce { SignalLogger.stop() },
            )
            .withName("Wrist SysId")

    init {
        SmartDashboard.putData(this)
        SmartDashboard.putData(sysId)
    }

    fun Angle.motorToWrist() = this / MOTOR_TO_WRIST

    fun Angle.wristToMotor() = this * MOTOR_TO_WRIST
}
