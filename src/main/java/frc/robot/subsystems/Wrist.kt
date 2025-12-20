package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.sim.ChassisReference.Clockwise_Positive
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.robot.lib.*
import org.littletonrobotics.junction.Logger

object Wrist {

    var lowerLimit = 11.25.degrees
    private var upperLimit = 197.degrees
    private const val COMP_GEAR_RATIO = (72 * 72 * 72 * 48) / (14 * 24 * 24 * 24.0)

    private val standardConfig =
        TalonFXConfiguration().apply {
            Feedback.SensorToMechanismRatio = COMP_GEAR_RATIO

            MotorOutput.withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive)

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
        }

    private val motionMagic =
        DynamicMotionMagicVoltage(
            0.degrees,
            1.rotationsPerSecond,
            3.rotationsPerSecondPerSecond,
            0.rotationsPerSecondCubed,
        )

    private val leader = TalonFX(13, "*").apply { configurator.apply(standardConfig) }

    val atPosition
        get() = leader.closedLoopError.value < 2.0

    fun goTo(angle: Angle) {
        leader.setControl(motionMagic.withPosition(angle))
    }

    val angle: Angle
        get() = leader.position.value

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

    fun simulationPeriodic() {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage())
        armSim.setInputVoltage(simState.motorVoltage)
        armSim.update(0.02)
        simState.setRawRotorPosition(armSim.angleRads.radians * COMP_GEAR_RATIO)
        simState.setRotorVelocity(armSim.velocityRadPerSec.radiansPerSecond * COMP_GEAR_RATIO)
    }

    fun periodic() {
        Logger.recordOutput("wrist/angle_degrees", angle.degrees)
        Logger.recordOutput("wrist/at_position", atPosition)
    }
}
