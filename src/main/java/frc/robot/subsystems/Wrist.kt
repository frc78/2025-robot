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
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
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
    private var upperLimit = 150.degrees
    private const val MOTOR_TO_WRIST = (72 * 72 * 72 * 48) / (14 * 24 * 24 * 16.0)

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

    private var currentSetpoint: Angle = leader.position.value
        set(value) {
            field = value
            angle
        }

    init {
        defaultCommand = run { leader.setControl(positionVoltage.withPosition(currentSetpoint)) }
    }

    val motionMagic = MotionMagicVoltage(0.degrees)
    val positionVoltage = PositionVoltage(0.degrees)
    val voltageOut = VoltageOut(0.0)

    fun goTo(state: RobotState): Command =
        PrintCommand("Wrist going to $state - ${state.wristAngle}")
            .alongWith(Commands.runOnce({ currentSetpoint = state.wristAngle }))

    val angle: Angle
        get() = currentSetpoint

    fun manualUp(): Command {
        return startEnd(
            { leader.setControl(voltageOut.withOutput(2.0.volts)) },
            { currentSetpoint = leader.position.value },
        )
    }

    fun manualDown(): Command {
        return startEnd(
            { leader.setControl(voltageOut.withOutput((-2.0).volts)) },
            { currentSetpoint = leader.position.value },
        )
    }

    val resetPosition: Command =
        Commands.runOnce({
            leader.setPosition(0.0)
            currentSetpoint = 0.degrees
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
        SmartDashboard.putData(this)
        SmartDashboard.putData(sysId)
    }

    private val wristSim =
        SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            MOTOR_TO_WRIST,
            1.0,
            .5,
            0.0,
            2.0,
            false,
            0.0,
        )

    private val motorSim by lazy { leader.simState }

    override fun simulationPeriodic() {
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage())
        wristSim.setInput(motorSim.motorVoltage)
        wristSim.update(0.02)
        motorSim.setRawRotorPosition(wristSim.angleRads * MOTOR_TO_WRIST)
        motorSim.setRotorVelocity(wristSim.velocityRadPerSec * MOTOR_TO_WRIST)
    }
}
