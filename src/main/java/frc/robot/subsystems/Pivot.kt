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
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
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

object Pivot : SubsystemBase("Pivot") {

    private const val GEAR_RATIO = (5.0 * 5 * 64 * 60) / (30 * 12)
    private val cancoder = CANcoder(5, "*")
    private val leader =
        TalonFX(9, "*").apply {
            val config =
                TalonFXConfiguration().apply {
                    MotorOutput.NeutralMode = NeutralModeValue.Brake
                    // Set soft limits to avoid breaking the pivot

                    SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(150.degrees)
                        .withReverseSoftLimitThreshold(0.degrees)
                    // Set feedback to encoder
                    Feedback.withFusedCANcoder(cancoder).withRotorToSensorRatio(GEAR_RATIO)
                    // Set feedforward and feedback gains
                    Slot0.withKP(38.77)
                        .withKD(14.947)
                        .withKS(0.13503)
                        .withKV(32.70)
                        .withKA(0.69849)
                        .withKG(0.2816)
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                    MotionMagic.MotionMagicCruiseVelocity = .25
                    MotionMagic.MotionMagicAcceleration = 2.5
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
        defaultCommand = Commands.idle(this)
    }

    fun goTo(state: RobotState): Command =
        PrintCommand("Pivot going to $state - ${state.pivotAngle}")
            .alongWith(runOnce { leader.setControl(motionMagic.withPosition(state.pivotAngle)) })

    val angle: Angle
        get() = cancoder.position.value

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
            { leader.setControl(voltageOut.withOutput(0.volts)) },
        )
    }

    val moveDown by command {
        startEnd(
            { leader.setControl(voltageOut.withOutput((-2).volts)) },
            { leader.setControl(voltageOut.withOutput(0.volts)) },
        )
    }
    private val sysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(
                .3.voltsPerSecond,
                3.volts,
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
                    leader.position.value >= 150.degrees
                },
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until {
                    leader.position.value <= 30.degrees
                },
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until {
                    leader.position.value >= 150.degrees
                },
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until {
                    leader.position.value <= 30.degrees
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

    init {
        SmartDashboard.putData(this)
        SmartDashboard.putData(sysId)
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
