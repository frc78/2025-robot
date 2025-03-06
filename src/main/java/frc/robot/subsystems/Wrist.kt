package frc.robot.subsystems

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.IS_COMP
import frc.robot.lib.amps
import frc.robot.lib.command
import frc.robot.lib.degrees
import frc.robot.lib.radians
import frc.robot.lib.radiansPerSecond
import frc.robot.lib.rotationsPerSecond
import frc.robot.lib.rotationsPerSecondCubed
import frc.robot.lib.rotationsPerSecondPerSecond
import frc.robot.lib.seconds
import frc.robot.lib.volts
import frc.robot.lib.voltsPerSecond
import kotlin.math.PI
import kotlin.math.abs
import org.littletonrobotics.junction.Logger

object Wrist : SubsystemBase("Wrist") {
    private var lowerLimit = 11.25.degrees
    private var upperLimit = 197.degrees
    private const val ALPHA_GEAR_RATIO = (72 * 72 * 64 * 48) / (14 * 24 * 32 * 16.0)
    private const val COMP_GEAR_RATIO = (72 * 72 * 72 * 48) / (14 * 24 * 24 * 16.0)

    private val SETPOINT_THRESHOLD = 3.degrees

    private val ALPHA_BOT_MOTOR_OUTPUT_CONFIG =
        MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive)

    private val COMP_BOT_MOTOR_OUTPUT_CONFIG =
        MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive)

    private val standardConfig =
        TalonFXConfiguration().apply {
            Feedback.SensorToMechanismRatio = if (IS_COMP) COMP_GEAR_RATIO else ALPHA_GEAR_RATIO

            MotorOutput =
                if (IS_COMP) COMP_BOT_MOTOR_OUTPUT_CONFIG else ALPHA_BOT_MOTOR_OUTPUT_CONFIG

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

    val motionMagic =
        DynamicMotionMagicVoltage(
            0.degrees,
            10.rotationsPerSecond,
            30.rotationsPerSecondPerSecond,
            100.rotationsPerSecondCubed,
        )

    private val leader = TalonFX(13, "*").apply { configurator.apply(standardConfig) }

    val atPosition
        get() = (leader.position.value - motionMagic.positionMeasure).abs(Degrees) < 1

    val voltageOut = VoltageOut(0.0)

    fun initializePosition() {
        if (leader.position.value < lowerLimit) {
            leader.setControl(motionMagic.withPosition(lowerLimit))
        }
    }

    fun isAtSetpoint(target: Angle): Boolean {
        return abs((angle - target).degrees) < SETPOINT_THRESHOLD.degrees
    }

    // Moves the wrist to <setpoint> and holds the command until <endCondition> is true
    fun goToRawUntil(setpoint: Angle, endCondition: () -> Boolean): Command =
        runOnce {
                motionMagic.withPosition(setpoint)
                if (Intake.detectAlgaeByCurrent()) {
                    leader.setControl(
                        motionMagic.withVelocity(3.0).withAcceleration(6.0).withJerk(30.0)
                    )
                } else {
                    leader.setControl(
                        motionMagic.withVelocity(10.0).withAcceleration(30.0).withJerk(100.0)
                    )
                }
            }
            .andThen(Commands.waitUntil(endCondition))

    fun goTo(state: RobotState): Command = goToRawUntil(state.wristAngle) { true }

    fun goToAndWaitUntilAtAngle(state: RobotState): Command =
        PrintCommand("Wrist waiting until it gets to $state - ${state.wristAngle}")
            .alongWith(goTo(state))
            .andThen(Commands.idle())
            .until { isAtSetpoint(state.wristAngle) }

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
        SmartDashboard.putData(sysId)
        //        SmartDashboard.putData("Zero wrist", resetPosition)
    }

    private val simState by lazy { leader.simState }
    private val armSim by lazy {
        SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            ALPHA_GEAR_RATIO,
            .07,
            .3,
            0.0,
            PI,
            false,
            0.00,
        )
    }

    override fun simulationPeriodic() {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage())
        armSim.setInputVoltage(simState.motorVoltage)
        armSim.update(0.02)
        simState.setRawRotorPosition(armSim.angleRads.radians * ALPHA_GEAR_RATIO)
        simState.setRotorVelocity(armSim.velocityRadPerSec.radiansPerSecond * ALPHA_GEAR_RATIO)
    }

    override fun periodic() {
        super.periodic()
        Logger.recordOutput("wrist/angle_degrees", angle.degrees)
        Logger.recordOutput("wrist/at_position", atPosition)
    }
}
