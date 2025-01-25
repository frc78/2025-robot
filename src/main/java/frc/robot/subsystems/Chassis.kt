package frc.robot.subsystems

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import org.littletonrobotics.junction.Logger

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
object Chassis :
    SwerveDrivetrain<TalonFX, TalonFX, CANcoder>(
        // TW: DeviceConstructor is a functional interface, meaning you can use a method reference
        // TW: In this case, the intention is to use the constructor of the class.
        // TW: To reference the constructor in kotlin, you can use `::ClassName`
        DeviceConstructor<TalonFX> { deviceId: Int, canbus: String? -> TalonFX(deviceId, canbus) },
        DeviceConstructor<TalonFX> { deviceId: Int, canbus: String? -> TalonFX(deviceId, canbus) },
        DeviceConstructor<CANcoder> { deviceId: Int, canbus: String? ->
            CANcoder(deviceId, canbus)
        },
        TunerConstants.DrivetrainConstants,
        0.0,
        Matrix(Nat.N3(), Nat.N1()),
        Matrix(Nat.N3(), Nat.N1()),
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight,
    ),
    Subsystem {

    /* Keep track if we've ever applied the operator perspective before or not */
    private var hasAppliedOperatorPerspective = false
    private val kBlueAlliancePerspectiveRotation: Rotation2d = Rotation2d.kZero
    private val kRedAlliancePerspectiveRotation: Rotation2d = Rotation2d.k180deg

    /* Swerve requests to apply during SysId characterization */
    private val translationCharacterization = SwerveRequest.SysIdSwerveTranslation()
    private val steerCharacterization = SwerveRequest.SysIdSwerveSteerGains()
    private val rotationCharacterization = SwerveRequest.SysIdSwerveRotation()

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private val sysIdRoutineTranslation =
        SysIdRoutine(
            SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Units.Volts.of(4.0), // Reduce dynamic step voltage to 4 V to prevent brownout
                null,
            ) // Use default timeout (10 s)
            // Log state with SignalLogger class
            { state: SysIdRoutineLog.State ->
                SignalLogger.writeString("SysIdTranslation_State", state.toString())
            },
            Mechanism(
                { output: Voltage? -> setControl(translationCharacterization.withVolts(output)) },
                null,
                this,
            ),
        )

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private val m_sysIdRoutineSteer =
        SysIdRoutine(
            SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Units.Volts.of(7.0), // Use dynamic voltage of 7 V
                null,
            ) // Use default timeout (10 s)
            // Log state with SignalLogger class
            { state: SysIdRoutineLog.State ->
                SignalLogger.writeString("SysIdSteer_State", state.toString())
            },
            Mechanism(
                { volts: Voltage? -> setControl(steerCharacterization.withVolts(volts)) },
                null,
                this,
            ),
        )

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private val sysIdRoutineRotation =
        SysIdRoutine(
            SysIdRoutine.Config(
                /* This is in radians per second², but SysId only supports "volts per second" */
                // TW: Try creating an extension property on the `Number` class that lets you write
                // this as `
                // TW: `(Math.PI / 6).volts`
                Units.Volts.of(Math.PI / 6)
                    .per(
                        Units.Second
                    ), /* This is in radians per second, but SysId only supports "volts" */
                Units.Volts.of(Math.PI),
                null,
            ) // Use default timeout (10 s)
            // Log state with SignalLogger class
            { state: SysIdRoutineLog.State ->
                SignalLogger.writeString("SysIdRotation_State", state.toString())
            },
            Mechanism(
                { output: Voltage ->
                    /* output is actually radians per second, but SysId only supports "volts" */
                    setControl(
                        rotationCharacterization.withRotationalRate(output.`in`(Units.Volts))
                    )
                    /* also log the requested output for SysId */
                    SignalLogger.writeDouble("Rotational_Rate", output.`in`(Units.Volts))
                },
                null,
                this,
            ),
        )

    /* The SysId routine to test */
    private val sysIdRoutineToApply = sysIdRoutineTranslation

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    fun applyRequest(requestSupplier: () -> SwerveRequest): Command {
        return run { this.setControl(requestSupplier()) }
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine specified by
     * [.m_sysIdRoutineToApply].
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command {
        return sysIdRoutineToApply.quasistatic(direction)
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine specified by
     * [.m_sysIdRoutineToApply].
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command {
        return sysIdRoutineToApply.dynamic(direction)
    }

    override fun periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent { allianceColor: Alliance ->
                setOperatorPerspectiveForward(
                    if (allianceColor == Alliance.Red) kRedAlliancePerspectiveRotation
                    else kBlueAlliancePerspectiveRotation
                )
                hasAppliedOperatorPerspective = true
            }
        }
        Vision.update()

        Logger.recordOutput("ChassisPose", state.Pose)
    }
}
