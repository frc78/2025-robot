package frc.robot.generated

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Voltage

object TestBotTunerConstants {
    // Both sets of gains need to be tuned to your individual frc.robot.
    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private val steerGains: Slot0Configs =
        // TW: An alternative, more concise, way to write this is to use the `apply` function
        Slot0Configs()
            .withKP(30.0)
            .withKI(0.0)
            .withKD(0.5)
            .withKS(0.1)
            .withKV(0.0)
            .withKA(0.0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)

    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private val driveGains: Slot0Configs =
        Slot0Configs().withKP(0.1).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.124)

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private val steerClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage

    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private val driveClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage

    // The type of motor used for the drive motor
    private val driveMotorType = DriveMotorArrangement.TalonFX_Integrated

    // The type of motor used for the drive motor
    private val steerMotorType = SteerMotorArrangement.TalonFX_Integrated

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    private val steerFeedbackType = SwerveModuleConstants.SteerFeedbackType.FusedCANcoder

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual frc.robot
    private val slipCurrent: Current = Units.Amps.of(120.0)

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private val driveInitialConfigs = TalonFXConfiguration()
    private val steerInitialConfigs: TalonFXConfiguration =
        TalonFXConfiguration()
            .withCurrentLimits(
                CurrentLimitsConfigs() // Swerve azimuth does not require much torque output, so we
                    // can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Units.Amps.of(60.0))
                    .withStatorCurrentLimitEnable(true)
            )
    private val encoderInitialConfigs = CANcoderConfiguration()

    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private val pigeonConfigs: Pigeon2Configuration? = null

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    private val CANBus: CANBus = CANBus("*")

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual frc.robot
    private val speedAt12Volts: LinearVelocity = Units.MetersPerSecond.of(5.32)

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual frc.robot
    private const val COUPLE_RATIO = 3.5714285714285716

    private const val DRIVE_GEAR_RATIO = 6.746031746031747
    private const val STEER_GEAR_RATIO = 21.428571428571427
    private val kWheelRadius: Distance = Units.Inches.of(2.25)

    private const val INVERT_LEFT = false
    private const val INVERT_RIGHT = true

    private const val PIGEON_ID = 0

    // These are only used for simulation
    private val steerInertia: MomentOfInertia = Units.KilogramSquareMeters.of(0.01)
    private val driveInertia: MomentOfInertia = Units.KilogramSquareMeters.of(0.01)

    // Simulated voltage necessary to overcome friction
    private val steerFrictionVoltage: Voltage = Units.Volts.of(0.2)
    private val driveFrictionVoltage: Voltage = Units.Volts.of(0.2)

    val DrivetrainConstants: SwerveDrivetrainConstants =
        SwerveDrivetrainConstants()
            .withCANBusName(CANBus.name)
            .withPigeon2Id(PIGEON_ID)
            .withPigeon2Configs(pigeonConfigs)

    private val ConstantCreator:
        SwerveModuleConstantsFactory<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration,
        > =
        SwerveModuleConstantsFactory<
                TalonFXConfiguration,
                TalonFXConfiguration,
                CANcoderConfiguration,
            >()
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLE_RATIO)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSlipCurrent(slipCurrent)
            .withSpeedAt12Volts(speedAt12Volts)
            .withDriveMotorType(driveMotorType)
            .withSteerMotorType(steerMotorType)
            .withFeedbackSource(steerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(steerInertia)
            .withDriveInertia(driveInertia)
            .withSteerFrictionVoltage(steerFrictionVoltage)
            .withDriveFrictionVoltage(driveFrictionVoltage)

    // Front Left
    private const val FL_DRIVE_ID = 1
    private const val FL_STEER_ID = 2
    private const val FL_ENCODER_ID = 1
    private val FL_ENCODER_OFFSET: Angle = Units.Rotations.of(0.365234375)
    private const val FL_STEER_INVERT = true
    private const val FL_ENCODER_INVERT = false

    private val FL_X_POS: Distance = Units.Inches.of(10.4)
    private val FL_Y_POS: Distance = Units.Inches.of(10.4)

    // Front Right
    private const val kFrontRightDriveMotorId = 3
    private const val kFrontRightSteerMotorId = 4
    private const val kFrontRightEncoderId = 2
    private val kFrontRightEncoderOffset: Angle = Units.Rotations.of(-0.04541015625)
    private const val kFrontRightSteerMotorInverted = true
    private const val kFrontRightEncoderInverted = false

    private val kFrontRightXPos: Distance = Units.Inches.of(10.4)
    private val kFrontRightYPos: Distance = Units.Inches.of(-10.4)

    // Back Left
    private const val kBackLeftDriveMotorId = 5
    private const val kBackLeftSteerMotorId = 6
    private const val kBackLeftEncoderId = 3
    private val kBackLeftEncoderOffset: Angle = Units.Rotations.of(-0.041259765625)
    private const val kBackLeftSteerMotorInverted = true
    private const val kBackLeftEncoderInverted = false

    private val kBackLeftXPos: Distance = Units.Inches.of(-10.4)
    private val kBackLeftYPos: Distance = Units.Inches.of(10.4)

    // Back Right
    private const val kBackRightDriveMotorId = 7
    private const val kBackRightSteerMotorId = 8
    private const val kBackRightEncoderId = 4
    private val kBackRightEncoderOffset: Angle = Units.Rotations.of(0.47607421875)
    private const val kBackRightSteerMotorInverted = true
    private const val kBackRightEncoderInverted = false

    private val kBackRightXPos: Distance = Units.Inches.of(-10.4)
    private val kBackRightYPos: Distance = Units.Inches.of(-10.4)

    val FrontLeft:
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            FL_STEER_ID,
            FL_DRIVE_ID,
            FL_ENCODER_ID,
            FL_ENCODER_OFFSET,
            FL_X_POS,
            FL_Y_POS,
            INVERT_LEFT,
            FL_STEER_INVERT,
            FL_ENCODER_INVERT,
        )
    val FrontRight:
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId,
            kFrontRightDriveMotorId,
            kFrontRightEncoderId,
            kFrontRightEncoderOffset,
            kFrontRightXPos,
            kFrontRightYPos,
            INVERT_RIGHT,
            kFrontRightSteerMotorInverted,
            kFrontRightEncoderInverted,
        )
    val BackLeft:
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId,
            kBackLeftDriveMotorId,
            kBackLeftEncoderId,
            kBackLeftEncoderOffset,
            kBackLeftXPos,
            kBackLeftYPos,
            INVERT_LEFT,
            kBackLeftSteerMotorInverted,
            kBackLeftEncoderInverted,
        )
    val BackRight:
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId,
            kBackRightDriveMotorId,
            kBackRightEncoderId,
            kBackRightEncoderOffset,
            kBackRightXPos,
            kBackRightYPos,
            INVERT_RIGHT,
            kBackRightSteerMotorInverted,
            kBackRightEncoderInverted,
        )
}
