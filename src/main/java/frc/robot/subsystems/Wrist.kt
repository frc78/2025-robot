package frc.robot.subsystems

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.lib.amps
import frc.robot.lib.degrees
import frc.robot.lib.seconds
import frc.robot.lib.volts
import frc.robot.lib.voltsPerSecond

object Wrist : SubsystemBase("Wrist") {
    private var lowerLimit = 0.degrees
    private var upperLimit = 120.degrees
    private const val GEAR_RATIO = (72 * 72 * 72 * 48) / (14 * 24 * 32 * 16.0)

    private val leader =
        TalonFX(13, "*").apply {
            val config =
                TalonFXConfiguration().apply {
                    Feedback.SensorToMechanismRatio = GEAR_RATIO

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

    //    private var currentSetpoint: Angle = leader.position.value

    val motionMagic = MotionMagicVoltage(0.degrees)
    val positionVoltage = PositionVoltage(0.degrees)
    val voltageOut = VoltageOut(0.0)

    fun goToRaw(setPoint: Angle): Command =
        Commands.runOnce({ Wrist.leader.setControl(Wrist.motionMagic.withPosition(setPoint)) })

    fun goTo(state: RobotState): Command =
        PrintCommand("Wrist going to $state - ${state.wristAngle}")
            //            .alongWith(Commands.runOnce({ currentSetpoint = state.wristAngle }))
            .alongWith(Commands.runOnce({ goToRaw(state.wristAngle) }))

    val angle: Angle
        get() = leader.position.value

    fun manualUp(): Command {
        return startEnd(
            { leader.setControl(voltageOut.withOutput(2.0.volts)) },
            //            { currentSetpoint = leader.position.value },
            { goToRaw(leader.position.value) },
        )
    }

    fun manualDown(): Command {
        return startEnd(
            { leader.setControl(voltageOut.withOutput((-2.0).volts)) },
            //            { currentSetpoint = leader.position.value },
            { goToRaw(leader.position.value) },
        )
    }

    val resetPosition: Command =
        Commands.runOnce({
            leader.setPosition(0.0)
            //            currentSetpoint = 0.degrees
        })

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
        //        defaultCommand = run {
        // leader.setControl(positionVoltage.withPosition(currentSetpoint)) }
        SmartDashboard.putData(this)
        SmartDashboard.putData(sysId)
    }
}
