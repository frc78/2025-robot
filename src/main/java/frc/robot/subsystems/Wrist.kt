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
import com.ctre.phoenix6.sim.ChassisReference.Clockwise_Positive
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.IS_COMP
import frc.robot.lib.FieldGeometry
import frc.robot.lib.command
import frc.robot.lib.degrees
import frc.robot.lib.meters
import frc.robot.lib.radians
import frc.robot.lib.radiansPerSecond
import frc.robot.lib.rotationsPerSecond
import frc.robot.lib.rotationsPerSecondCubed
import frc.robot.lib.rotationsPerSecondPerSecond
import frc.robot.lib.seconds
import frc.robot.lib.volts
import frc.robot.lib.voltsPerSecond
import frc.robot.subsystems.drivetrain.Chassis
import org.littletonrobotics.junction.Logger

object Wrist : SubsystemBase("wrist") {

    var lowerLimit = 11.25.degrees
    private var upperLimit = 197.degrees
    private const val ALPHA_GEAR_RATIO = (72 * 72 * 48) / (14 * 24 * 16.0)
    private const val COMP_GEAR_RATIO = (72 * 72 * 72 * 48) / (14 * 24 * 24 * 24.0)

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
                .withKI(0.0)
                .withKD(19.613)
                .withKS(0.21336)
                .withKV(10.335)
                .withKA(0.090409)
                .withKG(0.0)
                .withGravityType(GravityTypeValue.Arm_Cosine)

            MotionMagic.MotionMagicCruiseVelocity = 1.0
            MotionMagic.MotionMagicAcceleration = 100.0
            //            MotionMagic.MotionMagicJerk = 50.0
        }

    private var setpoint = lowerLimit
        set(value) {
            field = value.coerceIn(lowerLimit, upperLimit)
        }

    private val motionMagic =
        DynamicMotionMagicVoltage(
            0.degrees,
            10.rotationsPerSecond,
            30.rotationsPerSecondPerSecond,
            100.rotationsPerSecondCubed,
        )

    private val leader = TalonFX(13, "*").apply { configurator.apply(standardConfig) }

    private val shouldLimitReverseMotion: Boolean
        get() {
            return Pivot.angle < 20.degrees
        }

    val atPosition
        get() = (angle - setpoint).abs(Degrees) < 1

    private val voltageOut = VoltageOut(0.0)

    fun initializePosition() {
        if (leader.position.value < lowerLimit) {
            leader.setControl(motionMagic.withPosition(lowerLimit))
        }
    }

    fun goToWithoutRequiring(state: RobotState) = Commands.runOnce({ setpoint = state.wristAngle })

    fun goTo(state: RobotState): Command = runOnce {
        // do not move wrist if within 0.9 meters of a coral station
        // TODO change to limit reverse motion
        if (
            FieldGeometry.distanceToClosestLine(
                    FieldGeometry.CORAL_STATIONS,
                    Chassis.state.Pose.translation,
                )
                .meters >= 0.6.meters
        )
            setpoint = state.wristAngle
    }

    val angle: Angle
        get() = leader.position.value

    val manualUp by command { run { setpoint += 10.degrees * .020 } }

    val manualDown by command { run { setpoint -= 10.degrees * .020 } }

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

    private val simState by lazy { leader.simState.apply { Orientation = Clockwise_Positive } }
    private val armSim by lazy {
        SingleJointedArmSim(
            /* gearbox = */ DCMotor.getKrakenX60Foc(1),
            /* gearing = */ COMP_GEAR_RATIO,
            /* jKgMetersSquared = */ 0.0611,
            /* armLengthMeters = */ .3,
            /* minAngleRads = */ 0.0,
            /* maxAngleRads = */ upperLimit.radians,
            /* simulateGravity = */ false,
            /* startingAngleRads = */ 0.00,
        )
    }

    override fun simulationPeriodic() {
        armSim.setInputVoltage(simState.motorVoltage)
        armSim.update(0.02)
        simState.setRawRotorPosition(armSim.angleRads.radians * COMP_GEAR_RATIO)
        simState.setRotorVelocity(armSim.velocityRadPerSec.radiansPerSecond * COMP_GEAR_RATIO)
    }

    override fun periodic() {
        Logger.recordOutput("wrist/angle_degrees", angle.degrees)
        Logger.recordOutput("wrist/at_position", atPosition)
        Logger.recordOutput("wrist/setpoint", setpoint.degrees)

        // Command the wrist to move to the setpoint
        motionMagic.withPosition(setpoint).withLimitReverseMotion(shouldLimitReverseMotion)
        if (Intake.detectAlgaeByCurrent()) {
            motionMagic.withVelocity(3.0).withAcceleration(6.0).withJerk(30.0)
        } else {
            motionMagic.withVelocity(10.0).withAcceleration(30.0).withJerk(100.0)
        }
        leader.setControl(motionMagic)
    }
}
