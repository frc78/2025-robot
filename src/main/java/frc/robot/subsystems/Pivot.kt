package frc.robot.subsystems

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import com.ctre.phoenix6.sim.ChassisReference
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.IS_COMP
import frc.robot.lib.command
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.lib.meters
import frc.robot.lib.radians
import frc.robot.lib.radiansPerSecond
import frc.robot.lib.seconds
import frc.robot.lib.volts
import frc.robot.lib.voltsPerSecond
import kotlin.math.abs
import org.littletonrobotics.junction.Logger

object Pivot : Subsystem {

    private const val GEAR_RATIO = (5.0 * 5 * 64 * 60) / (30 * 12) // 266.25
    private val cancoder = CANcoder(5, "*")

    // how close pivot needs to be to its setpoint for goToAndWaitUntilVertical to terminate
    private val SETPOINT_THRESHOLD = 4.degrees
    // how vertical the pivot needs to be for the elevator to extend
    private val RAISE_ELEVATOR_THRESHOLD = 60.degrees
    // how horizontal the pivot needs to be for the
    val EXTEND_FOOT_THRESHOLD = 20.degrees

    private val ALPHA_BOT_SLOT0_CONFIGS =
        Slot0Configs()
            .withKP(65.365) // 24.365
            .withKI(0.1)
            .withKD(0.22908)
            .withKS(0.1755)
            .withKV(31.983)
            .withKA(0.49753)
            .withKG(0.22628)

    private val COMP_BOT_SLOT0_CONFIGS =
        Slot0Configs()
            .withKP(100.0)
            .withKI(0.0)
            .withKD(0.29431)
            .withKS(0.24723)
            .withKV(29.598)
            .withKA(0.42529)
            .withKG(0.0082199)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)

    // Used when going to coral station
    private val COMP_BOT_SLOT1_CONFIGS =
        Slot1Configs()
            .withKP(200.0)
            .withKI(0.0)
            .withKD(0.29431)
            .withKS(0.24723)
            .withKV(29.598)
            .withKA(0.42529)
            .withKG(0.0082199)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)

    private val COMP_BOT_FEEDBACK_CONFIGS =
        FeedbackConfigs().withFusedCANcoder(cancoder).withRotorToSensorRatio(GEAR_RATIO)
    private val ALPHA_BOT_FEEDBACK_CONFIGS =
        FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO)

    private val leader =
        TalonFX(9, "*").apply {
            val config =
                TalonFXConfiguration().apply {
                    MotorOutput.NeutralMode = NeutralModeValue.Brake
                    // Set soft limits to avoid breaking the pivot

                    SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(160.degrees)
                        .withReverseSoftLimitThreshold(0.degrees)

                    Feedback =
                        if (IS_COMP) COMP_BOT_FEEDBACK_CONFIGS else ALPHA_BOT_FEEDBACK_CONFIGS
                    // Set feedforward and feedback gains
                    Slot0 = if (IS_COMP) COMP_BOT_SLOT0_CONFIGS else ALPHA_BOT_SLOT0_CONFIGS
                    Slot1 = COMP_BOT_SLOT1_CONFIGS
                    MotionMagic.MotionMagicCruiseVelocity = .25
                    MotionMagic.MotionMagicAcceleration = .5
                    MotionMagic.MotionMagicJerk = 2.5
                }

            configurator.apply(config)
        }
    private val follower =
        TalonFX(10, "*").apply {
            configurator.apply(MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
        }

    private val motionMagic = MotionMagicVoltage(0.degrees)

    init {
        follower.setControl(Follower(9, true))
    }

    // Checks if the pivot is sufficiently vertical to extend the elevator
    val canExtendElevator: Boolean
        get() = angle > RAISE_ELEVATOR_THRESHOLD

    fun isAtSetpoint(target: Angle): Boolean {
        return abs((angle - target).degrees) < SETPOINT_THRESHOLD.degrees
    }

    // Moves the pivot to <setpoint> and holds the command until <endCondition> is true
    fun goToRawUntil(setpoint: Angle, endCondition: () -> Boolean): Command =
        run {
                leader.setControl(
                    motionMagic
                        .withPosition(setpoint)
                        .withLimitForwardMotion(Climber.isExtended)
                        .withSlot(if (Elevator.position < 10.inches) 1 else 0)
                )
            }
            .until(endCondition)

    val atPosition
        get() = (leader.position.value - motionMagic.positionMeasure).abs(Degrees) < 1

    fun goTo(state: RobotState): Command = goToRawUntil(state.pivotAngle) { true }

    val angle: Angle
        get() = leader.position.value

    // Only create this object when it is needed during simulation
    private val pivotSim by lazy {
        SingleJointedArmSim(
            /* gearbox = */ DCMotor.getKrakenX60Foc(2),
            /* gearing = */ GEAR_RATIO,
            /* jKgMetersSquared = */ 2.730,
            /* armLengthMeters = */ 40.inches.meters,
            /* minAngleRads = */ 0.degrees.radians,
            /* maxAngleRads = */ 180.degrees.radians,
            /* simulateGravity = */ true,
            /* startingAngleRads = */ 45.degrees.radians,
        )
    }

    private val leaderSimState by lazy { leader.simState }
    private val encoderSim by lazy {
        cancoder.simState.apply { Orientation = ChassisReference.CounterClockwise_Positive }
    }

    private val voltageOut = VoltageOut(0.0)
    val moveUp by command {
        startEnd(
            {
                leader.setControl(
                    voltageOut.withOutput(2.volts).withLimitForwardMotion(Climber.isExtended)
                )
            },
            { leader.setControl(motionMagic.withPosition(leader.position.value)) },
        )
    }

    val moveDown by command {
        startEnd(
            { leader.setControl(voltageOut.withOutput((-2).volts)) },
            { leader.setControl(motionMagic.withPosition(leader.position.value)) },
        )
    }
    private val sysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(
                1.voltsPerSecond,
                7.volts,
                10.seconds,
                { SignalLogger.writeString("pivot_state", "$it") },
            ),
            SysIdRoutine.Mechanism(
                { leader.setControl(voltageOut.withOutput(it)) },
                null,
                this,
                "pivot",
            ),
        )

    fun coast() {
        leader.setNeutralMode(NeutralModeValue.Coast)
        follower.setNeutralMode(NeutralModeValue.Coast)
    }

    fun brake() {
        leader.setNeutralMode(NeutralModeValue.Brake)
        follower.setNeutralMode(NeutralModeValue.Brake)
    }

    val sysId =
        Commands.sequence(
                runOnce { SignalLogger.start() },
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until {
                    leader.position.value >= 160.degrees
                },
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until {
                    leader.position.value <= 10.degrees
                },
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until {
                    leader.position.value >= 160.degrees
                },
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until {
                    leader.position.value <= 10.degrees
                },
                runOnce { SignalLogger.stop() },
            )
            .withName("Pivot SysId")

    override fun periodic() {
        Logger.recordOutput("pivot/angle_degrees", angle.degrees)
        Logger.recordOutput("pivot/at_position", atPosition)
    }

    override fun simulationPeriodic() {
        leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage())
        encoderSim.setSupplyVoltage(RobotController.getBatteryVoltage())

        pivotSim.setInputVoltage(leaderSimState.motorVoltage)
        pivotSim.update(0.020)
        encoderSim.setRawPosition(pivotSim.angleRads.radians)
        encoderSim.setVelocity(pivotSim.velocityRadPerSec.radiansPerSecond)

        leaderSimState.setForwardLimit(pivotSim.hasHitUpperLimit())
        leaderSimState.setReverseLimit(pivotSim.hasHitLowerLimit())
        leaderSimState.setRawRotorPosition((pivotSim.angleRads * GEAR_RATIO).radians)
        leaderSimState.setRotorVelocity((pivotSim.velocityRadPerSec * GEAR_RATIO).radiansPerSecond)
    }
}
