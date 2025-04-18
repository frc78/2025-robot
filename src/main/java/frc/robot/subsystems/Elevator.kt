package frc.robot.subsystems

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import com.ctre.phoenix6.sim.ChassisReference
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.IS_COMP
import frc.robot.lib.command
import frc.robot.lib.inches
import frc.robot.lib.kilograms
import frc.robot.lib.meters
import frc.robot.lib.metersPerSecond
import frc.robot.lib.pounds
import frc.robot.lib.rotations
import frc.robot.lib.toAngle
import frc.robot.lib.toAngularVelocity
import frc.robot.lib.toDistance
import frc.robot.lib.volts
import org.littletonrobotics.junction.Logger

object Elevator : SubsystemBase("elevator") {
    private val motionMagic = MotionMagicVoltage(0.0)
    val IS_STOWED_THRESHOLD = 3.inches
    val MOVE_PIVOT_THRESHOLD = 40.inches

    private const val LEADER_MOTOR_ID = 11
    private const val FOLLOWER_MOTOR_ID = 12
    // Constants for the feedforward calculation
    private val ALPHA_BOT_CONFIGS =
        Slot0Configs().apply {
            kS = 0.23487
            kV = 0.60823
            kA = 0.034044
            kG = 0.55356

            kP = 34.887
            kI = 0.0
            kD = 1.2611
        }
    private val COMP_BOT_CONFIGS =
        Slot0Configs().apply {
            kS = 0.21739
            kV = 0.56149
            kA = 0.013467
            kG = 0.32537

            kP = 10.614
            kI = 0.0
            kD = 0.26475
        }

    private const val GEAR_RATIO = 5.0
    private val DRUM_RADIUS = (1.75.inches + .25.inches) / 2.0

    val MAX_HEIGHT = if (IS_COMP) 54.inches else 54.inches

    private val leader =
        TalonFX(LEADER_MOTOR_ID, "*").apply {
            val leaderMotorConfiguration =
                TalonFXConfiguration().apply {
                    Feedback.SensorToMechanismRatio = GEAR_RATIO
                    SoftwareLimitSwitch.ForwardSoftLimitEnable = true
                    // Do not allow the motor to move upwards until after zeroing
                    SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                        MAX_HEIGHT.toDrumRotations().rotations
                    // Allow the motor to move downwards until the current limit is reached
                    SoftwareLimitSwitch.ReverseSoftLimitEnable = false
                    SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0

                    MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
                    MotorOutput.NeutralMode = NeutralModeValue.Brake

                    Slot0 = if (IS_COMP) COMP_BOT_CONFIGS else ALPHA_BOT_CONFIGS
                    Slot0.GravityType = GravityTypeValue.Elevator_Static
                    Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign

                    MotionMagic.withMotionMagicCruiseVelocity(18.0)
                        .withMotionMagicAcceleration(400.0)
                        .withMotionMagicJerk(1000.0)
                }
            configurator.apply(leaderMotorConfiguration)
            position.setUpdateFrequency(100.0)
            velocity.setUpdateFrequency(100.0)
            motorVoltage.setUpdateFrequency(100.0)
            closedLoopError.setUpdateFrequency(100.0)
        }

    val position
        get() = leader.position.value.toElevatorHeight()

    var setpoint = 0.inches
        set(value) {
            field = value.coerceIn(0.inches, MAX_HEIGHT)
        }

    fun goTo(state: RobotState): Command = Commands.runOnce({ setpoint = state.elevatorHeight })

    fun goToRaw(height: Distance): Command = runOnce { setpoint = height }

    val isStowed: Boolean
        get() = position < IS_STOWED_THRESHOLD

    val atPosition: Boolean
        get() = (position - setpoint).abs(Inches) < .5

    val manualUp by command { run { setpoint += 10.inches * 0.020 } }

    val manualDown by command { run { setpoint -= 10.inches * 0.020 } }

    init {
        TalonFX(FOLLOWER_MOTOR_ID, "*").apply { setControl(Follower(LEADER_MOTOR_ID, true)) }
    }

    private fun Distance.toDrumRotations() = this.toAngle(DRUM_RADIUS)

    private fun Angle.toElevatorHeight() = this.toDistance(DRUM_RADIUS)

    private val elevatorSim by lazy {
        ElevatorSim(
            DCMotor.getKrakenX60Foc(2),
            5.0,
            35.132.pounds.kilograms,
            DRUM_RADIUS.meters,
            0.0.inches.meters,
            MAX_HEIGHT.meters,
            true,
            5.inches.meters,
        )
    }
    private val leaderSim by lazy {
        leader.simState.apply { Orientation = ChassisReference.CounterClockwise_Positive }
    }

    private val voltageOut = VoltageOut(0.volts)

    private val sysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(null, 3.volts, null) {
                SignalLogger.writeString("elevator_state", "$it")
            },
            SysIdRoutine.Mechanism(
                { leader.setControl(voltageOut.withOutput(it)) },
                null,
                this,
                "elevator",
            ),
        )

    @Suppress("UnusedPrivateProperty")
    private val sysId by command {
        Commands.sequence(
                runOnce {
                    SignalLogger.start()
                    leader.configurator.apply(
                        SoftwareLimitSwitchConfigs().apply {
                            ForwardSoftLimitEnable = true
                            ForwardSoftLimitThreshold = MAX_HEIGHT.toDrumRotations().rotations
                            ReverseSoftLimitEnable = true
                            ReverseSoftLimitThreshold = 0.1
                        }
                    )
                },
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until {
                    leader.position.value > 50.inches.toDrumRotations()
                },
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until {
                    leader.position.value < 6.inches.toDrumRotations()
                },
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until {
                    leader.position.value > 50.inches.toDrumRotations()
                },
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until {
                    leader.position.value < 6.inches.toDrumRotations()
                },
                runOnce { SignalLogger.stop() },
            )
            .withName("Elevator sysId")
    }

    override fun periodic() {
        Logger.recordOutput("elevator/position", position.inches)
        Logger.recordOutput("elevator/setpoint", setpoint.inches)
        Logger.recordOutput("elevator/stowed", isStowed)
        Logger.recordOutput("elevator/at_position", atPosition)
        leader.setControl(motionMagic.withPosition(setpoint.toDrumRotations()))
    }

    override fun simulationPeriodic() {
        elevatorSim.setInputVoltage(RobotController.getBatteryVoltage())
        val motorVoltage = leaderSim.motorVoltage
        elevatorSim.setInputVoltage(motorVoltage)
        elevatorSim.update(0.02)
        leaderSim.setRawRotorPosition(
            elevatorSim.positionMeters.meters.toAngle(DRUM_RADIUS) * GEAR_RATIO
        )
        leaderSim.setRotorVelocity(
            elevatorSim.velocityMetersPerSecond.metersPerSecond.toAngularVelocity(DRUM_RADIUS) *
                GEAR_RATIO
        )
    }
}
