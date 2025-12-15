package frc.robot.subsystems

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.robot.lib.*
import org.littletonrobotics.junction.Logger

object Pivot {

    private const val GEAR_RATIO = (5.0 * 5 * 64 * 60) / (30 * 12) // 266.25

    private val UPPER_LIMIT = 160.degrees
    private val LOWER_LIMIT = 0.degrees

    private val leader =
        TalonFX(9, "*").apply {
            val config =
                TalonFXConfiguration().apply {
                    MotorOutput.NeutralMode = NeutralModeValue.Brake
                    // Set soft limits to avoid breaking the pivot

                    SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(UPPER_LIMIT)
                        .withReverseSoftLimitThreshold(LOWER_LIMIT)

                    Feedback.withSensorToMechanismRatio(GEAR_RATIO)
                    // Set feedforward and feedback gains
                    Slot0.withKP(180.0) // 200
                        .withKI(0.0)
                        .withKD(0.2) // 0.29431
                        .withKS(0.24723)
                        .withKV(29.598)
                        .withKA(0.42529)
                        .withKG(0.0082199)
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                    Slot1.withKP(50.0)
                        .withKI(0.0)
                        .withKD(0.29431)
                        .withKS(0.24723)
                        .withKV(29.598)
                        .withKA(0.42529)
                        .withKG(0.0082199)
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                    MotionMagic.MotionMagicCruiseVelocity = .25
                    MotionMagic.MotionMagicAcceleration = 1.0 // .5
                    MotionMagic.MotionMagicJerk = 3.5 // 2.5
                }

            configurator.apply(config)
        }
    private val follower =
        TalonFX(10, "*").apply {
            configurator.apply(MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            setControl(Follower(9, true))
        }

    private val motionMagic = MotionMagicVoltage(0.0)

    val angle: Angle
        get() = leader.position.value

    fun goTo(angle: Angle) {
        leader.setControl(motionMagic.withPosition(angle))
    }

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

    fun coast() {
        leader.setNeutralMode(NeutralModeValue.Coast)
        follower.setNeutralMode(NeutralModeValue.Coast)
    }

    fun brake() {
        leader.setNeutralMode(NeutralModeValue.Brake)
        follower.setNeutralMode(NeutralModeValue.Brake)
    }

    fun periodic() {
        Logger.recordOutput("pivot/angle_degrees", angle.degrees)
    }

    fun simulationPeriodic() {
        pivotSim.setInputVoltage(leaderSimState.motorVoltage)
        pivotSim.update(0.020)
        leaderSimState.setRawRotorPosition(pivotSim.angleRads.radians * GEAR_RATIO)
        leaderSimState.setRotorVelocity(pivotSim.velocityRadPerSec.radiansPerSecond * GEAR_RATIO)
        leaderSimState.setForwardLimit(pivotSim.hasHitUpperLimit())
        leaderSimState.setReverseLimit(pivotSim.hasHitLowerLimit())
    }
}
