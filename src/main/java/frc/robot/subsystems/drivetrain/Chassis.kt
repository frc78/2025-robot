package frc.robot.subsystems.drivetrain

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.util.DriveFeedforwards
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.system.plant.DCMotor
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
import edu.wpi.first.wpilibj2.command.DeferredCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.IS_COMP
import frc.robot.Robot
import frc.robot.generated.CompBotTunerConstants
import frc.robot.generated.TunerConstants
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain
import frc.robot.lib.FieldGeometry
import frc.robot.lib.FieldPoses.closestBarge
import frc.robot.lib.FieldPoses.closestBranch
import frc.robot.lib.FieldPoses.closestCoralStation
import frc.robot.lib.FieldPoses.closestLeftBarge
import frc.robot.lib.FieldPoses.closestLeftBranch
import frc.robot.lib.FieldPoses.closestProcessor
import frc.robot.lib.FieldPoses.closestReef
import frc.robot.lib.FieldPoses.closestRightBarge
import frc.robot.lib.FieldPoses.closestRightBranch
import frc.robot.lib.SysIdSwerveTranslationTorqueCurrentFOC
import frc.robot.lib.amps
import frc.robot.lib.command
import frc.robot.lib.inches
import frc.robot.lib.kilogramSquareMeters
import frc.robot.lib.meters
import frc.robot.lib.metersPerSecond
import frc.robot.lib.metersPerSecondPerSecond
import frc.robot.lib.pounds
import frc.robot.lib.radians
import frc.robot.lib.rotateByAlliance
import frc.robot.lib.rotationsPerSecond
import frc.robot.lib.rotationsPerSecondPerSecond
import frc.robot.lib.seconds
import frc.robot.lib.volts
import frc.robot.lib.voltsPerSecond
import frc.robot.subsystems.Intake
import java.io.IOException
import java.text.ParseException
import kotlin.math.PI
import kotlin.math.hypot
import org.littletonrobotics.junction.Logger

private val drivetrainConstants =
    if (IS_COMP) CompBotTunerConstants.DrivetrainConstants else TunerConstants.DrivetrainConstants
private val frontLeft = if (IS_COMP) CompBotTunerConstants.FrontLeft else TunerConstants.FrontLeft
private val frontRight =
    if (IS_COMP) CompBotTunerConstants.FrontRight else TunerConstants.FrontRight
private val backLeft = if (IS_COMP) CompBotTunerConstants.BackLeft else TunerConstants.BackLeft
private val backRight = if (IS_COMP) CompBotTunerConstants.BackRight else TunerConstants.BackRight

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
@Suppress("UnusedPrivateProperty", "TooManyFunctions")
object Chassis :
    TunerSwerveDrivetrain(drivetrainConstants, 0.0, frontLeft, frontRight, backLeft, backRight),
    Subsystem {

    private val table = NetworkTableInstance.getDefault().getTable("drivetrain")
    private val closestReefPub = table.getStructTopic("closest_reef", Pose2d.struct).publish()
    private val closestBranchPub = table.getStructTopic("closest_branch", Pose2d.struct).publish()
    private val closestCoralStationPub =
        table.getStructTopic("closest_coral", Pose2d.struct).publish()
    private val closestProcessorPub =
        table.getStructTopic("closest_processor", Pose2d.struct).publish()
    private val closestRightCoralStationPub =
        table.getStructTopic("closest_coral_station_right", Pose2d.struct).publish()
    private val closestLeftCoralStationPub =
        table.getStructTopic("closest_coral_station_left", Pose2d.struct).publish()

    private val closestBargePub = table.getStructTopic("closest_barge", Pose2d.struct).publish()
    private val closestBargeLeftPub =
        table.getStructTopic("closest_barge_left", Pose2d.struct).publish()
    private val closestBargeRightPub =
        table.getStructTopic("closest_barge_right", Pose2d.struct).publish()

    init {
        // This would normally be called by SubsystemBase, but since we cannot extend that class,
        // we call manually
        CommandScheduler.getInstance().registerSubsystem(this)
        if (Utils.isSimulation()) {
            startSimThread()
        }

        PathPlannerLogging.setLogTargetPoseCallback {
            Logger.recordOutput("pathfind_to_pose_target", it)
        }

        @Suppress("SpreadOperator")
        PathPlannerLogging.setLogActivePathCallback {
            Logger.recordOutput("pathfind_to_pose_path", *it.toTypedArray())
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
    private val translationCharacterizationCurrent = SysIdSwerveTranslationTorqueCurrentFOC()
    private val steerCharacterization = SwerveRequest.SysIdSwerveSteerGains()
    private val rotationCharacterization = SwerveRequest.SysIdSwerveRotation()

    private val pathApplyRobotSpeeds =
        ApplyRobotSpeeds().withDriveRequestType(SwerveModule.DriveRequestType.Velocity)

    // For use with commands which are still taking driver input for translation
    private val FieldCentricFacingAngleDriver: SwerveRequest.FieldCentricFacingAngle =
        SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(6.0, 0.0, 0.1)
            .withRotationalDeadband(0.05)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

    // For use with alignment commands
    private val FieldCentricFacingAngleAlignments: SwerveRequest.FieldCentricFacingAngle =
        SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(6.0, 0.0, 0.1)
            .withRotationalDeadband(0.05)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

    private val FieldCentric =
        SwerveRequest.FieldCentric()
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

    private val RobotRelative =
        SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

    fun configureAutoBuilder() {
        try {
            val config =
                RobotConfig(
                    116.pounds,
                    3.123.kilogramSquareMeters,
                    ModuleConfig(
                        2.inches,
                        4.48.metersPerSecond,
                        1.2,
                        DCMotor.getKrakenX60Foc(1),
                        7.13,
                        80.amps,
                        1,
                    ),
                    Translation2d(frontLeft.LocationX, frontLeft.LocationY),
                    Translation2d(frontRight.LocationX, frontRight.LocationY),
                    Translation2d(backLeft.LocationX, backLeft.LocationY),
                    Translation2d(backRight.LocationX, backRight.LocationY),
                )
            AutoBuilder.configure(
                { state.Pose }, // Supplier of current robot pose
                this::resetPose, // Consumer for seeding pose against auto
                { state.Speeds }, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                { speeds: ChassisSpeeds, feedforwards: DriveFeedforwards ->
                    setControl(
                        pathApplyRobotSpeeds
                            .withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                    )
                },
                PPHolonomicDriveController( // PID constants for translation
                    PIDConstants(20.0, 0.0, 0.0), // PID constants for rotation
                    PIDConstants(7.0, 0.0, 0.0),
                ),
                config,
                { Robot.alliance == Alliance.Red },
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
                5.0.volts,
                2.seconds,
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

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private val sysIdRoutineTranslationCurrent =
        SysIdRoutine(
            SysIdRoutine.Config(
                6.voltsPerSecond, // Use default ramp rate (1 A/s)
                10.0.volts,
                4.seconds,
            ) // Use default timeout (10 s)
            // Log state with SignalLogger class
            { state: SysIdRoutineLog.State ->
                SignalLogger.writeString("SysIdTranslation_State", state.toString())
            },
            Mechanism(
                { output: Voltage ->
                    // output is actually amps, but SysId only supports "volts"
                    setControl(translationCharacterizationCurrent.withCurrent(output.volts))
                },
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
    private val sysIdRoutineToApply = sysIdRoutineTranslationCurrent

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    private fun applyRequest(requestSupplier: () -> SwerveRequest): Command {
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

    val sysIdRoutine by command {
        runOnce { SignalLogger.start() }
            .andThen(sysIdQuasistatic(SysIdRoutine.Direction.kForward))
            .andThen(Commands.waitSeconds(1.0))
            .andThen(sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(Commands.waitSeconds(1.0))
            .andThen(sysIdDynamic(SysIdRoutine.Direction.kForward))
            .andThen(Commands.waitSeconds(1.0))
            .andThen(sysIdDynamic(SysIdRoutine.Direction.kReverse))
            .andThen(Commands.waitSeconds(1.0))
            .finallyDo { _ -> SignalLogger.stop() }
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
        closestProcessorPub.set(closestProcessor)

        closestBargePub.set(closestBarge)
        closestBargeLeftPub.set(closestLeftBarge)
        closestBargeRightPub.set(closestRightBarge)
    }

    /** Drives to a pose such that the coral is at x=0 */
    fun driveToPoseWithCoralOffset(pose: () -> Pose2d) = driveToPose {
        pose().transformBy(Transform2d(0.inches, -Intake.coralLocation, Rotation2d.kZero))
    }

    private fun pathfindToPose(pose: () -> Pose2d) =
        DeferredCommand(
            {
                if (state.Pose.translation.getDistance(pose().translation) < 1.0) {
                    driveToPose(pose)
                } else {
                    AutoBuilder.pathfindToPose(
                        // Stay back 1 foot from the end point
                        pose(), // .transformBy(Transform2d(.3.meters, 0.meters, Rotation2d.kZero)),
                        PathConstraints(
                            2.5.metersPerSecond,
                            8.metersPerSecondPerSecond,
                            1.rotationsPerSecond,
                            10.rotationsPerSecondPerSecond,
                            12.0.volts,
                        ),
                    )
                }
            },
            setOf(this),
        )

    private val poseController =
        ProfiledPIDController(0.5, 0.0, 0.05, TrapezoidProfile.Constraints(4.0, 2.3)).apply {
            goal = TrapezoidProfile.State(0.0, 0.0)
        }

    private fun primeDriveToPose(pose: () -> Pose2d): Command =
        Commands.runOnce({
            val target = pose()
            val diff = target.minus(state.Pose)
            val distance = diff.translation.norm
            distanceFromPoseGoal = distance
            poseController.reset(distance, 0.0)

            Logger.recordOutput("DriveToPose target", target)
            FieldCentricFacingAngleAlignments.withTargetDirection(target.rotation)
            hasPoseTarget = true
        })

    private val isAtPIDGoal: Boolean
        get() =
            poseController.atGoal() &&
                FieldCentricFacingAngleAlignments.HeadingController.atSetpoint()

    private var distanceFromPoseGoal = 0.0
    private var hasPoseTarget = false

    fun isWithinGoal(distance: Double) = hasPoseTarget && distanceFromPoseGoal < distance

    private fun driveToPose(pose: () -> Pose2d): Command =
        primeDriveToPose(pose)
            .andThen(
                applyRequest {
                        val robot = Chassis.state.Pose
                        val target = pose()
                        val diff = robot.translation - target.translation

                        distanceFromPoseGoal = diff.norm
                        val output = poseController.calculate(diff.norm)

                        val angle = diff.angle
                        val xSpeed = (poseController.setpoint.velocity + output) * angle.cos
                        val ySpeed = (poseController.setpoint.velocity + output) * angle.sin

                        FieldCentricFacingAngleAlignments.withVelocityX(xSpeed)
                            .withVelocityY(ySpeed)
                    }
                    .until { poseController.atGoal() }
            )
            // Stop movement
            .finallyDo { _ ->
                setControl(ApplyRobotSpeeds())
                hasPoseTarget = false
            }

    private fun driveToChangingPose(
        pose: () -> Pose2d,
        strafeOverride: () -> ChassisSpeeds,
        endWhenGoal: Boolean,
    ): Command =
        primeDriveToPose(pose)
            .andThen(
                applyRequest {
                        val robot = Chassis.state.Pose
                        val requestedDriveDelta = strafeOverride()

                        val target = pose()
                        val diff = robot.translation - target.translation

                        val output = poseController.calculate(diff.norm)

                        val angle = diff.angle
                        val xSpeed = (poseController.setpoint.velocity + output) * angle.cos
                        val ySpeed = (poseController.setpoint.velocity + output) * angle.sin

                        FieldCentricFacingAngleAlignments.withVelocityX(xSpeed)
                            .withVelocityY(ySpeed)
                            .withTargetDirection(
                                FieldCentricFacingAngleAlignments.TargetDirection.plus(
                                    Rotation2d.fromRadians(
                                        requestedDriveDelta.omegaRadiansPerSecond
                                    )
                                )
                            )
                    }
                    .raceWith(
                        Commands.waitUntil { endWhenGoal && isAtPIDGoal }
                            .andThen(Commands.waitSeconds(0.5))
                            .andThen(Commands.waitUntil { endWhenGoal && isAtPIDGoal })
                    )
            )

            // Stop movement
            .finallyDo { _ -> setControl(ApplyRobotSpeeds()) }

    val driveToClosestReef by command { driveToPose { closestReef } }

    val driveToLeftBranch by command {
        driveToPoseWithCoralOffset { closestLeftBranch }.withName("Drive to branch left")
    }

    val driveToRightBranch by command {
        driveToPoseWithCoralOffset { closestRightBranch }.withName("Drive to branch right")
    }

    val driveToProcessor by command { driveToPose { closestProcessor } }

    val driveToClosestCenterCoralStation by command { driveToPose { closestCoralStation } }

    val driveToBarge by command { driveToPose { closestBarge } }
    val driveToBargeLeft by command { driveToPose { closestLeftBarge } }
    val driveToBargeRight by command { driveToPose { closestRightBarge } }

    fun snapAngleToReef(
        block: SwerveRequest.FieldCentricFacingAngle.() -> SwerveRequest.FieldCentricFacingAngle
    ): Command {
        return applyRequest {
            FieldCentricFacingAngleDriver.withTargetDirection(
                    closestReef.rotation.rotateByAlliance()
                )
                .block()
        }
    }

    fun snapAngleToCoralStation(
        block: SwerveRequest.FieldCentricFacingAngle.() -> SwerveRequest.FieldCentricFacingAngle
    ): Command {
        return applyRequest {
            FieldCentricFacingAngleDriver.withTargetDirection(
                    closestCoralStation.rotation.rotateByAlliance()
                )
                .block()
        }
    }

    fun fieldCentricDrive(
        block: SwerveRequest.FieldCentric.() -> SwerveRequest.FieldCentric
    ): Command {
        return applyRequest { FieldCentric.block() }
    }

    fun robotCentricDrive(block: SwerveRequest.RobotCentric.() -> SwerveRequest.RobotCentric) =
        applyRequest { RobotRelative.block() }.finallyDo { _ -> setControl(ApplyRobotSpeeds()) }

    fun driveToClosestSubstation(
        strafeSpeedY: () -> Double,
        withSpeeds: SwerveRequest.RobotCentric.() -> SwerveRequest.RobotCentric,
        distanceToSubstation: Double,
    ): Command =
        // Our target distance from the line segment of the substation
        driveToChangingPose(
                {
                    val position = Chassis.state.Pose.translation
                    // Gets closest substation line segment
                    val closestSubstation =
                        FieldGeometry.getClosestLine(FieldGeometry.CORAL_STATIONS, position)
                    // Gets the vector to the closest point on that line segment. SDF gives the
                    // vector from the point to the robot, so it needs an unary minus and then added
                    // to the robot position
                    val closestSubstationPoint =
                        closestSubstation
                            .getVectorFromClosestPoint(position)
                            .unaryMinus()
                            .plus(position)
                    // Calculates a translation that is going to offset the point on the line
                    // segment to get the goal position
                    val offsetTranslation =
                        closestSubstation.getPerpendicularUnitVector().times(distanceToSubstation)
                    // Translates the point on the line segment by the offset to get the goal
                    // position
                    Pose2d(
                            closestSubstationPoint.plus(offsetTranslation),
                            closestSubstation.getPerpendicularUnitVector().unaryMinus().angle,
                        )
                        .also { Logger.recordOutput("Drive to closest substation", it) }
                },
                {
                    // Any way to be able to use the variables from above?
                    val speedTranslation =
                        FieldGeometry.getClosestLine(
                                FieldGeometry.CORAL_STATIONS,
                                state.Pose.translation,
                            )
                            .getParallelUnitVector() * strafeSpeedY()
                    ChassisSpeeds(speedTranslation.x, speedTranslation.y, 0.0).also {
                        Logger.recordOutput("Drive changing velocity", it)
                    }
                },
                true,
            )
            .andThen(applyRequest { RobotRelative.withSpeeds() })

    fun driveToBarge(
        strafeSpeedY: () -> Double,
        withSpeeds: SwerveRequest.RobotCentric.() -> SwerveRequest.RobotCentric,
    ): Command =
        // Our target distance from the line segment of the substation
        driveToChangingPose(
                {
                    val position = state.Pose.translation
                    // Gets position of closes point on the alignment line
                    val closestBargePoint =
                        FieldGeometry.getClosestLine(FieldGeometry.BARGE_ALIGNMENT_LINES, position)
                            .getVectorFromClosestPoint(position)
                            .unaryMinus()
                            .plus(position)
                    Pose2d(
                            closestBargePoint,
                            if (position.x <= FieldGeometry.FIELD_X_LENGTH / 2) Rotation2d.k180deg
                            else Rotation2d.kZero,
                        )
                        .also { Logger.recordOutput("Drive to barge", it) }
                },
                {
                    // Any way to be able to use the variables from above?
                    val speedTranslation =
                        FieldGeometry.getClosestLine(
                                FieldGeometry.BARGE_ALIGNMENT_LINES,
                                Chassis.state.Pose.translation,
                            )
                            .getParallelUnitVector() * strafeSpeedY()
                    ChassisSpeeds(speedTranslation.x, speedTranslation.y, 0.0).also {
                        Logger.recordOutput("Drive ", it)
                    }
                },
                true,
            )
            .andThen(applyRequest { RobotRelative.withSpeeds() })

    val measureWheelRotations by command {
        val startYaw = pigeon2.yaw.value
        val startPositions = state.ModulePositions.map { it.distanceMeters }
        run {
            setControl(RobotRelative.withRotationalRate(.25.rotationsPerSecond))
            val angularDisplacement = pigeon2.yaw.value - startYaw
            val linearDisplacement =
                angularDisplacement.radians * this.moduleLocations[0].let { hypot(it.x, it.y) }
            val wheelRotations =
                startPositions.mapIndexed { index, startPos ->
                    (state.ModulePositions[index].distanceMeters - startPos) /
                        (CompBotTunerConstants.kWheelRadius.meters * PI * 2)
                }
            val wheelRadii =
                wheelRotations.map { linearDisplacement / (it * 2 * PI) }.toDoubleArray()
            Logger.recordOutput("module_radius", wheelRadii)
        }
    }
}
