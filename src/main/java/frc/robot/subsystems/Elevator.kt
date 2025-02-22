package frc.robot.subsystems

import com.ctre.phoenix6.SignalLogger
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
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.lib.*

object Elevator : SubsystemBase("Elevator") {
    private val motionMagic = MotionMagicVoltage(0.0)
    private val voltage = VoltageOut(0.0)
    val IS_STOWED_THRESHOLD = 3.inches

    fun goTo(state: RobotState): Command =
        PrintCommand("Elevator going to $state - ${state.elevatorHeight}")
            .alongWith(
                runOnce {
                    leader.setControl(
                        motionMagic
                            .withPosition(state.elevatorHeight.toAngle(DRUM_RADIUS))
                            .withSlot(0)
                            .withEnableFOC(true)
                    )
                }
            )

    val isStowed: Boolean = position < IS_STOWED_THRESHOLD

    fun goToAndWaitUntilStowed(state: RobotState): Command =
        PrintCommand("Elevator going to $state - ${state.elevatorHeight}")
            .alongWith(
                runOnce {
                    leader.setControl(
                        motionMagic
                            .withPosition(state.elevatorHeight.toAngle(DRUM_RADIUS))
                            .withSlot(0)
                            .withEnableFOC(true)
                    )
                }
            )
            .andThen(Commands.idle())
            .until { isStowed || state.elevatorHeight > IS_STOWED_THRESHOLD }

    val manualUp by command {
        startEnd(
            { leader.setControl(voltage.withOutput(2.0.volts)) },
            { leader.setControl(voltage.withOutput(0.0.volts)) },
        )
    }

    val manualDown by command {
        startEnd(
            { leader.setControl(voltage.withOutput((-2.0).volts)) },
            { leader.setControl(voltage.withOutput(0.0.volts)) },
        )
    }

    val position
        get() = leader.position.value.toElevatorHeight()

    // Constants for the feedforward calculation
    private const val K_S = 0.23487
    private const val K_V = 0.60823
    private const val K_A = 0.034044
    private const val K_G = 0.55356

    // PID gains
    private const val K_P = 34.887
    private const val K_I = 0.0
    private const val K_D = 1.2611

    private const val LEADER_MOTOR_ID = 11
    private const val FOLLOWER_MOTOR_ID = 12

    private const val GEAR_RATIO = 5.0
    private val DRUM_RADIUS = (1.75.inches + .25.inches) / 2.0

    private val MAX_HEIGHT = 53.inches

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

                    MotorOutput.Inverted = InvertedValue.Clockwise_Positive
                    MotorOutput.NeutralMode = NeutralModeValue.Brake

                    Slot0.kS = K_S
                    Slot0.kV = K_V
                    Slot0.kA = K_A
                    Slot0.kG = K_G
                    Slot0.kP = K_P
                    Slot0.kI = K_I
                    Slot0.kD = K_D
                    Slot0.GravityType = GravityTypeValue.Elevator_Static
                    Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign

                    MotionMagic.withMotionMagicCruiseVelocity((119.17 * 0.5).radiansPerSecond)
                        .withMotionMagicAcceleration((898.69 * 0.5).radiansPerSecondPerSecond)
                }
            configurator.apply(leaderMotorConfiguration)
            position.setUpdateFrequency(100.0)
            velocity.setUpdateFrequency(100.0)
            motorVoltage.setUpdateFrequency(100.0)
            closedLoopError.setUpdateFrequency(100.0)
        }

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
            53.inches.meters,
            true,
            5.inches.meters,
        )
    }
    private val leaderSim by lazy {
        leader.simState.apply { Orientation = ChassisReference.Clockwise_Positive }
    }

    private val voltageOut = VoltageOut(0.volts)

    private val sysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(null, null, null, { SignalLogger.writeString("state", "$it") }),
            SysIdRoutine.Mechanism(
                { leader.setControl(voltageOut.withOutput(it)) },
                null,
                this,
                "elevator",
            ),
        )

    private val sysId by command {
        Commands.sequence(
                runOnce {
                    SignalLogger.start()
                    leader.configurator.apply(
                        SoftwareLimitSwitchConfigs().apply {
                            ForwardSoftLimitEnable = true
                            ForwardSoftLimitThreshold = MAX_HEIGHT.toDrumRotations().rotations
                            ReverseSoftLimitEnable = true
                            ReverseSoftLimitThreshold = 0.0
                        }
                    )
                },
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until {
                    leader.position.value > 40.inches.toDrumRotations()
                },
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until {
                    leader.position.value < 6.inches.toDrumRotations()
                },
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until {
                    leader.position.value > 40.inches.toDrumRotations()
                },
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until {
                    leader.position.value < 6.inches.toDrumRotations()
                },
                runOnce { SignalLogger.stop() },
            )
            .withName("Elevator sysId")
    }

    init {
        SmartDashboard.putData(sysId)
    }

    override fun simulationPeriodic() {
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
