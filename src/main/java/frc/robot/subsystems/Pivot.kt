package frc.robot.subsystems

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.sim.ChassisReference
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.lib.command
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.lib.meters
import frc.robot.lib.radians
import frc.robot.lib.radiansPerSecond
import frc.robot.lib.seconds
import frc.robot.lib.volts
import frc.robot.lib.voltsPerSecond
import java.util.function.BooleanSupplier
import kotlin.math.abs
import org.littletonrobotics.junction.Logger

object Pivot : SubsystemBase("Pivot") {

    private const val GEAR_RATIO = (5.0 * 5 * 64 * 60) / (30 * 12) // 266.25
    private val cancoder = CANcoder(5, "*")

    // how close pivot needs to be to its setpoint for goToAndWaitUntilVertical to terminate
    private val SETPOINT_THRESHOLD = 4.degrees
    // how vertical the pivot needs to be for the elevator to extend
    private val RAISE_ELEVATOR_THRESHOLD = 60.degrees
    // how horizontal the pivot needs to be for the
    val EXTEND_FOOT_THRESHOLD = 10.degrees

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
                    // Set feedback to encoder
                    // TODO encoder slipping, so zeroing manually for now on devbot
                    //
                    // Feedback.withFusedCANcoder(cancoder).withRotorToSensorRatio(GEAR_RATIO)
                    Feedback.SensorToMechanismRatio = GEAR_RATIO
                    // Set feedforward and feedback gains
                    Slot0.withKP(50.365) // 24.365
                        .withKI(0.1)
                        .withKD(0.22908)
                        .withKS(0.1755)
                        .withKV(31.983)
                        .withKA(0.49753)
                        .withKG(0.22628)
                        .withGravityType(GravityTypeValue.Arm_Cosine)
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
    fun goToRawUntil(setpoint: Angle, endCondition: BooleanSupplier): Command =
        runOnce { leader.setControl(motionMagic.withPosition(setpoint)) }
            .andThen(Commands.idle())
            .until(endCondition)

    val atPosition
        get() = (leader.position.value - motionMagic.positionMeasure).abs(Degrees) < 1

    fun goTo(state: RobotState): Command =
        PrintCommand("Pivot going to $state - ${state.pivotAngle}")
            .alongWith(runOnce { leader.setControl(motionMagic.withPosition(state.pivotAngle)) })

    fun goToAndWaitUntilSetpoint(state: RobotState): Command =
        PrintCommand("Pivot going vertical").alongWith(goTo(state)).andThen(Commands.idle()).until {
            abs((angle - state.pivotAngle).degrees) < SETPOINT_THRESHOLD.degrees
        }

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
            { leader.setControl(voltageOut.withOutput(2.volts)) },
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
                    leader.position.value >= 10.degrees
                },
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until {
                    leader.position.value <= 10.degrees
                },
                runOnce { SignalLogger.stop() },
            )
            .withName("Pivot SysId")

    val manualUp by command {
        startEnd(
            { leader.setControl(voltageOut.withOutput(2.volts)) },
            { leader.setControl(voltageOut.withOutput(0.volts)) },
        )
    }

    val manualDown by command {
        startEnd(
            { leader.setControl(voltageOut.withOutput((-2).volts)) },
            { leader.setControl(voltageOut.withOutput(0.volts)) },
        )
    }

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
