package frc.robot.subsystems.drivetrain

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.util.DriveFeedforwards
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.IS_TEST
import frc.robot.generated.TestBotTunerConstants
import frc.robot.generated.TunerConstants
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain
import frc.robot.lib.Alignments.REEF_TO_BRANCH_LEFT
import frc.robot.lib.Alignments.REEF_TO_BRANCH_RIGHT
import frc.robot.lib.Alignments.closestBranch
import frc.robot.lib.Alignments.closestCoralStation
import frc.robot.lib.Alignments.closestReef
import frc.robot.lib.Branch
import frc.robot.lib.ScoreSelector.SelectedBranch
import frc.robot.lib.command
import frc.robot.lib.feetPerSecond
import frc.robot.lib.inches
import frc.robot.lib.meters
import frc.robot.lib.metersPerSecond
import frc.robot.lib.rotationsPerSecond
import frc.robot.lib.volts
import frc.robot.lib.voltsPerSecond
import frc.robot.subsystems.Intake
import java.io.IOException
import java.text.ParseException
import kotlin.math.PI
import org.littletonrobotics.junction.Logger

val drivetrainConstants =
    if (IS_TEST) TestBotTunerConstants.DrivetrainConstants else TunerConstants.DrivetrainConstants
val frontLeft = if (IS_TEST) TestBotTunerConstants.FrontLeft else TunerConstants.FrontLeft
val frontRight = if (IS_TEST) TestBotTunerConstants.FrontRight else TunerConstants.FrontRight
val backLeft = if (IS_TEST) TestBotTunerConstants.BackLeft else TunerConstants.BackLeft
val backRight = if (IS_TEST) TestBotTunerConstants.BackRight else TunerConstants.BackRight

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
object Chassis :
    TunerSwerveDrivetrain(drivetrainConstants, 0.0, frontLeft, frontRight, backLeft, backRight),
    Subsystem {

    private val table = NetworkTableInstance.getDefault().getTable("drivetrain")
    private val closestReefPub = table.getStructTopic("closest_reef", Pose2d.struct).publish()
    private val closestBranchPub = table.getStructTopic("closest_branch", Pose2d.struct).publish()
    private val closestCoralStationPub =
        table.getStructTopic("closest_coral", Pose2d.struct).publish()

    init {
        // This would normally be called by SubsystemBase, but since we cannot extend that class,
        // we call manually
        CommandScheduler.getInstance().registerSubsystem(this)
        if (Utils.isSimulation()) {
            startSimThread()
        }
    }

    private var lastSimTime = Utils.getCurrentTimeSeconds()
    private lateinit var simNotifier: Notifier
    private const val SIM_LOOP_PERIOD = 0.005

    private fun startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds()
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = Notifier {
            val currentTime = Utils.getCurrentTimeSeconds()
            val deltaTime = currentTime - lastSimTime
            lastSimTime = currentTime

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage())
        }
        simNotifier.startPeriodic(SIM_LOOP_PERIOD)
    }

    /* Keep track if we've ever applied the operator perspective before or not */
    private var hasAppliedOperatorPerspective = false
    val kBlueAlliancePerspectiveRotation: Rotation2d = Rotation2d.kZero
    val kRedAlliancePerspectiveRotation: Rotation2d = Rotation2d.k180deg

    /* Swerve requests to apply during SysId characterization */
    private val translationCharacterization = SwerveRequest.SysIdSwerveTranslation()
    private val steerCharacterization = SwerveRequest.SysIdSwerveSteerGains()
    private val rotationCharacterization = SwerveRequest.SysIdSwerveRotation()

    private val pathApplyRobotSpeeds = ApplyRobotSpeeds()

    val FieldCentricFacingAngle =
        SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
            .withHeadingPID(6.0, 0.0, 0.1)
            .withRotationalDeadband(0.05)

    val FieldCentric =
        SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

    // Create these once to avoid memory allocation
    private val StrafeLeft =
        ApplyRobotSpeeds()
            .withSpeeds(
                ChassisSpeeds(0.0.metersPerSecond, 1.0.feetPerSecond, 0.0.rotationsPerSecond)
            )
    private val StrafeRight =
        ApplyRobotSpeeds()
            .withSpeeds(
                ChassisSpeeds(0.0.metersPerSecond, (-1.0).feetPerSecond, 0.0.rotationsPerSecond)
            )

    fun configureAutoBuilder() {
        try {
            val config = RobotConfig.fromGUISettings()
            AutoBuilder.configure(
                { state.Pose }, // Supplier of current robot pose
                this::resetPose, // Consumer for seeding pose against auto
                { state.Speeds }, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                { speeds: ChassisSpeeds, feedforwards: DriveFeedforwards ->
                    Chassis.setControl(
                        pathApplyRobotSpeeds
                            .withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                    )
                },
                PPHolonomicDriveController( // PID constants for translation
                    PIDConstants(10.0, 0.0, 0.0), // PID constants for rotation
                    PIDConstants(7.0, 0.0, 0.0),
                ),
                config,
                { DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red },
                this, // Subsystem for requirements
            )
        } catch (ex: IOException) {
            DriverStation.reportError(
                "Failed to load PathPlanner config and configure AutoBuilder",
                ex.stackTrace,
            )
        } catch (ex: ParseException) {
            DriverStation.reportError(
                "Failed to load PathPlanner config and configure AutoBuilder",
                ex.stackTrace,
            )
        }
    }

    override fun addVisionMeasurement(
        visionRobotPoseMeters: Pose2d,
        timestampSeconds: Double,
        visionMeasurementStdDevs: Matrix<N3, N1>,
    ) {
        super.addVisionMeasurement(
            visionRobotPoseMeters,
            Utils.fpgaToCurrentTime(timestampSeconds),
            visionMeasurementStdDevs,
        )
    }

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private val sysIdRoutineTranslation =
        SysIdRoutine(
            SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                4.0.volts,
                null,
            ) // Use default timeout (10 s)
            // Log state with SignalLogger class
            { state: SysIdRoutineLog.State ->
                SignalLogger.writeString("SysIdTranslation_State", state.toString())
            },
            Mechanism(
                { output: Voltage -> setControl(translationCharacterization.withVolts(output)) },
                null,
                this,
            ),
        )

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    @Suppress("UnusedPrivateProperty")
    private val m_sysIdRoutineSteer =
        SysIdRoutine(
            SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                7.0.volts,
                null,
            ) // Use default timeout (10 s)
            // Log state with SignalLogger class
            { state: SysIdRoutineLog.State ->
                SignalLogger.writeString("SysIdSteer_State", state.toString())
            },
            Mechanism(
                { volts: Voltage -> setControl(steerCharacterization.withVolts(volts)) },
                null,
                this,
            ),
        )

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    @Suppress("UnusedPrivateProperty")
    private val sysIdRoutineRotation =
        SysIdRoutine(
            SysIdRoutine.Config(
                /* This is in radians per second², but SysId only supports "volts per second" */
                (PI / 6).voltsPerSecond,
                PI.volts,
                null,
            ) // Use default timeout (10 s)
            // Log state with SignalLogger class
            { state: SysIdRoutineLog.State ->
                SignalLogger.writeString("SysIdRotation_State", state.toString())
            },
            Mechanism(
                { output: Voltage ->
                    /* output is actually radians per second, but SysId only supports "volts" */
                    setControl(rotationCharacterization.withRotationalRate(output.volts))
                    /* also log the requested output for SysId */
                    SignalLogger.writeDouble("Rotational_Rate", output.volts)
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
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    fun applyRequest(requestSupplier: () -> SwerveRequest): Command {
        return run { setControl(requestSupplier()) }
    }

    val zeroHeading: Command = Commands.runOnce({ resetRotation(Chassis.operatorForwardDirection) })

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
        closestReefPub.set(closestReef)
        closestBranchPub.set(closestBranch)
        closestCoralStationPub.set(closestCoralStation)
    }

    private val xController =
        ProfiledPIDController(
            10.0,
            0.0,
            0.0,
            TrapezoidProfile.Constraints(TunerConstants.kSpeedAt12Volts.metersPerSecond, 3.5),
        )
    private val yController =
        ProfiledPIDController(
            10.0,
            0.0,
            0.0,
            TrapezoidProfile.Constraints(TunerConstants.kSpeedAt12Volts.metersPerSecond, 3.5),
        )

    fun driveToPose(pose: () -> Pose2d): Command =
        Commands.runOnce({
                val fieldRelative =
                    ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.rotation)
                xController.reset(state.Pose.translation.x, fieldRelative.vxMetersPerSecond)
                yController.reset(state.Pose.translation.y, fieldRelative.vyMetersPerSecond)
                val target = pose()
                Logger.recordOutput("DriveToPose target", target)
                xController.goal = TrapezoidProfile.State(target.x, 0.0)
                yController.goal = TrapezoidProfile.State(target.y, 0.0)
                FieldCentricFacingAngle.withTargetDirection(target.rotation)
            })
            .andThen(
                applyRequest {
                    val robot = Chassis.state.Pose

                    val xOutput = xController.calculate(robot.x)
                    val yOutput = yController.calculate(robot.y)
                    Logger.recordOutput("DriveToPose xOutput", xOutput)
                    Logger.recordOutput("DriveToPose yOutput", yOutput)
                    FieldCentricFacingAngle.withVelocityX(xOutput).withVelocityY(yOutput)
                }
            )
            .until {
                xController.atGoal() &&
                    yController.atGoal() &&
                    FieldCentricFacingAngle.HeadingController.atSetpoint()
            }

    val driveToClosestReef by command { driveToPose { closestReef } }

    val driveToLeftBranch by command {
        driveToPose {
                closestReef
                    .transformBy(REEF_TO_BRANCH_LEFT)
                    .transformBy(Transform2d(0.inches, -Intake.coralLocation, Rotation2d.kZero))
            }
            .withName("Drive to branch left")
    }

    val driveToRightBranch by command {
        driveToPose {
                closestReef
                    .transformBy(REEF_TO_BRANCH_RIGHT)
                    .transformBy((Transform2d(0.inches, -Intake.coralLocation, Rotation2d.kZero)))
            }
            .withName("Drive to branch right")
    }

    val driveToClosestBranch by command { driveToPose { closestBranch } }

    val driveToSelectedBranch by command {
        ConditionalCommand(driveToLeftBranch, driveToRightBranch, { SelectedBranch == Branch.LEFT })
    }

    fun snapToReef(
        block: SwerveRequest.FieldCentricFacingAngle.() -> SwerveRequest.FieldCentricFacingAngle
    ): Command {
        return applyRequest {
            FieldCentricFacingAngle.withTargetDirection(closestReef.rotation).block()
        }
    }

    fun fieldCentricDrive(
        block: SwerveRequest.FieldCentric.() -> SwerveRequest.FieldCentric
    ): Command {
        return applyRequest { FieldCentric.block() }
    }

    val strafeLeft by command { applyRequest { StrafeLeft } }
    val strafeRight by command { applyRequest { StrafeRight } }

     val driveToClosestCoralStation by command {
        driveToPose {
                closestCoralStation.transformBy(Transform2d(0.5.meters, 0.meters, Rotation2d.kPi))
            }
            .withName("Drive to coral station")
    }

    init {
        //        SmartDashboard.putData(driveToRightBranch)
        //        SmartDashboard.putData(driveToLeftBranch)
        //        SmartDashboard.putData(driveToClosestBranch)
        //        SmartDashboard.putData(driveToClosestReef)
        //        SmartDashboard.putData(driveToClosestCoralStation)
    }
}
