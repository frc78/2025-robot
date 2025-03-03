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
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.DeferredCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.lib.amps
import frc.robot.lib.command
import frc.robot.lib.degrees
import frc.robot.lib.radians
import frc.robot.lib.radiansPerSecond
import frc.robot.lib.seconds
import frc.robot.lib.volts
import frc.robot.lib.voltsPerSecond
import java.util.function.BooleanSupplier
import kotlin.math.PI
import kotlin.math.abs
import org.littletonrobotics.junction.Logger

object Wrist : SubsystemBase("Wrist") {
    private var lowerLimit = 10.degrees
    private var upperLimit = 175.degrees
    private const val GEAR_RATIO = (72 * 72 * 72 * 48) / (14 * 24 * 32 * 16.0)
    private val AT_ANGLE_THRESHOLD = 3.degrees

    private val standardConfig =
        TalonFXConfiguration().apply {
            Feedback.SensorToMechanismRatio = GEAR_RATIO

            MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Brake

            SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(upperLimit)
                .withReverseSoftLimitThreshold(lowerLimit)

            Slot0.withKP(162.105) // 62.105
                .withKI(1.0)
                .withKD(19.613)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .withKG(0.0)
                .withGravityType(GravityTypeValue.Arm_Cosine)

            MotionMagic.MotionMagicCruiseVelocity = 10.0
            MotionMagic.MotionMagicAcceleration = 30.0
            MotionMagic.MotionMagicJerk = 100.0
        }

    private val hasAlgaeConfig =
        TalonFXConfiguration().apply {
            Feedback.SensorToMechanismRatio = GEAR_RATIO

            MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Brake

            SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(upperLimit)
                .withReverseSoftLimitThreshold(lowerLimit)

            Slot0.withKP(162.105) // 62.105
                .withKI(1.0)
                .withKD(19.613)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .withKG(0.0)
                .withGravityType(GravityTypeValue.Arm_Cosine)

            MotionMagic.MotionMagicCruiseVelocity = 2.0
            MotionMagic.MotionMagicAcceleration = 6.0
            MotionMagic.MotionMagicJerk = 30.0
        }

    private val leader = TalonFX(13, "*").apply { configurator.apply(standardConfig) }

    val motionMagic = MotionMagicVoltage(0.degrees)

    val atPosition
        get() = (leader.position.value - motionMagic.positionMeasure).abs(Degrees) < 1

    val positionVoltage = PositionVoltage(0.degrees)
    val voltageOut = VoltageOut(0.0)

    fun initializePosition() {
        leader.setControl(motionMagic.withPosition(lowerLimit))
    }

    fun isAtAngle(target: Angle): Boolean {
        return abs((angle - target).degrees) < AT_ANGLE_THRESHOLD.degrees
    }

    // Moves the wrist to <setpoint> and holds the command until <endCondition> is true
    fun goToRawUntil(setpoint: Angle, endCondition: BooleanSupplier): Command =
        runOnce { leader.setControl(motionMagic.withPosition(setpoint)) }
            .andThen(Commands.idle())
            .until(endCondition)

    fun goTo(state: RobotState): Command =
        DeferredCommand(
            {
                if (Intake.detectAlgaeByCurrent()) {
                    runOnce { leader.configurator.apply(hasAlgaeConfig) }
                        .andThen(
                            runOnce {
                                leader.setControl(motionMagic.withPosition(state.wristAngle))
                            }
                        )
                } else {
                    PrintCommand("Wrist going to $state - ${state.wristAngle}")
                        .alongWith(runOnce { leader.configurator.apply(standardConfig) })
                        .andThen(
                            runOnce {
                                leader.setControl(motionMagic.withPosition(state.wristAngle))
                            }
                        )
                }
            },
            setOf(Wrist),
        )

    fun goToAndWaitUntilAtAngle(state: RobotState): Command =
        PrintCommand("Wrist waiting until it gets to $state - ${state.wristAngle}")
            .alongWith(goTo(state))
            .andThen(Commands.idle())
            .until { isAtAngle(state.wristAngle) }

    val angle: Angle
        get() = leader.position.value

    fun manualUp(): Command {
        return startEnd(
            { leader.setControl(voltageOut.withOutput(2.0.volts)) },
            { leader.setControl(motionMagic.withPosition(leader.position.value)) },
        )
    }

    fun manualDown(): Command {
        return startEnd(
            { leader.setControl(voltageOut.withOutput((-2.0).volts)) },
            { leader.setControl(motionMagic.withPosition(leader.position.value)) },
        )
    }

    @Suppress("UnusedPrivateProperty")
    private val resetPosition by command { Commands.runOnce({ leader.setPosition(0.0) }) }

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

    @Suppress("UnusedPrivateProperty")
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
        //        SmartDashboard.putData(this)
        //        SmartDashboard.putData(sysId)
        //        SmartDashboard.putData("Zero wrist", resetPosition)
    }

    private val simState by lazy { leader.simState }
    private val armSim by lazy {
        SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), GEAR_RATIO, .07, .3, 0.0, PI, false, 0.00)
    }

    override fun simulationPeriodic() {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage())
        armSim.setInputVoltage(simState.motorVoltage)
        armSim.update(0.02)
        simState.setRawRotorPosition(armSim.angleRads.radians * GEAR_RATIO)
        simState.setRotorVelocity(armSim.velocityRadPerSec.radiansPerSecond * GEAR_RATIO)
    }

    override fun periodic() {
        super.periodic()
        Logger.recordOutput("wrist/angle_degrees", angle.degrees)
        Logger.recordOutput("wrist/at_position", atPosition)
    }
}
