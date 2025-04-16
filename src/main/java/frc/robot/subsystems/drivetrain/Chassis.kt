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
import com.pathplanner.lib.path.ConstraintsZone
import com.pathplanner.lib.path.EventMarker
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.DriveFeedforwards
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.IS_COMP
import frc.robot.Robot
import frc.robot.generated.CompBotTunerConstants
import frc.robot.generated.TunerConstants
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain
import frc.robot.lib.FieldPoses.REEF_TO_ROBOT_BACK_TRANSFORM
import frc.robot.lib.FieldPoses.REEF_TO_ROBOT_FRONT_TRANSFORM
import frc.robot.lib.FieldPoses.closestAlgae
import frc.robot.lib.FieldPoses.closestBarge
import frc.robot.lib.FieldPoses.closestCoralStation
import frc.robot.lib.FieldPoses.closestLeftBarge
import frc.robot.lib.FieldPoses.closestLeftBranch
import frc.robot.lib.FieldPoses.closestProcessor
import frc.robot.lib.FieldPoses.closestReef
import frc.robot.lib.FieldPoses.closestRightBarge
import frc.robot.lib.FieldPoses.closestRightBranch
import frc.robot.lib.Level
import frc.robot.lib.Level.L1
import frc.robot.lib.Level.L2
import frc.robot.lib.Level.L3
import frc.robot.lib.Level.L4
import frc.robot.lib.ScoreSelector.SelectedLevel
import frc.robot.lib.SysIdSwerveTranslationTorqueCurrentFOC
import frc.robot.lib.amps
import frc.robot.lib.command
import frc.robot.lib.inches
import frc.robot.lib.meters
import frc.robot.lib.metersPerSecond
import frc.robot.lib.metersPerSecondPerSecond
import frc.robot.lib.poundSquareInches
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
    private val closestAlgaePub = table.getStructTopic("closest_algae", Pose2d.struct).publish()
    private val closestLeftBranchPub =
        table.getStructTopic("closest_left_branch", Pose2d.struct).publish()
    private val closestRightBranchPub =
        table.getStructTopic("closest_right_branch", Pose2d.struct).publish()
    private val closestCoralStationPub =
        table.getStructTopic("closest_coral", Pose2d.struct).publish()
    private val closestProcessorPub =
        table.getStructTopic("closest_processor", Pose2d.struct).publish()

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
                    23009.poundSquareInches,
                    ModuleConfig(
                        2.inches,
                        4.48.metersPerSecond,
                        1.2,
                        DCMotor.getKrakenX60Foc(1),
                        7.13,
                        50.amps,
                        1,
                    ),
                    Translation2d(frontLeft.LocationX, frontLeft.LocationY),
                    Translation2d(frontRight.LocationX, frontRight.LocationY),
                    Translation2d(backLeft.LocationX, backLeft.LocationY),
                    Translation2d(backRight.LocationX, backRight.LocationY),
                )
            AutoBuilder.configure(
                /* poseSupplier = */ { state.Pose },
                /* resetPose = */ this::resetPose,
                /* robotRelativeSpeedsSupplier = */ { state.Speeds },
                /* output = */ { speeds: ChassisSpeeds, feedforwards: DriveFeedforwards ->
                    setControl(pathApplyRobotSpeeds.withSpeeds(speeds))
                },
                /* controller = */ PPHolonomicDriveController(
                    /* translationConstants = */ PIDConstants(5.0, 0.0, 0.1),
                    /* rotationConstants = */ PIDConstants(10.0, 0.0, 0.0),
                ),
                /* robotConfig = */ config,
                /* shouldFlipPath = */ { Robot.alliance == Alliance.Red },
                /* ...driveRequirements = */ this, // Subsystem for requirements
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
        closestAlgaePub.set(closestAlgae)
        closestLeftBranchPub.set(closestLeftBranch)
        closestRightBranchPub.set(closestRightBranch)
        closestCoralStationPub.set(closestCoralStation)
        closestProcessorPub.set(closestProcessor)

        closestBargePub.set(closestBarge)
        closestBargeLeftPub.set(closestLeftBarge)
        closestBargeRightPub.set(closestRightBarge)
    }

    /** Drives to a pose such that the coral is at x=0 */
    fun driveToPoseWithCoralOffset(pose: () -> Pose2d) = driveToPose {
        pose()
            .transformBy(Transform2d(0.inches, -Intake.coralLocation, Rotation2d.kZero))
            .transformBy(bestReefToBotTransform())
    }

    /** Drives to a pose such that the coral is at x=0 */
    fun pathplanToPoseWithCoralOffset(pose: () -> Pose2d) =
        pathplanToPose({ SelectedLevel != Level.L1 }) {
            pose()
                .transformBy(Transform2d(0.inches, -Intake.coralLocation, Rotation2d.kZero))
                .transformBy(bestReefToBotTransform())
        }

    var targetPose: Pose2d? = null
        private set

    /** Returns true if robot is within distance of a path planner drive sequence */
    private fun isPathPlannerWithinGoal(distance: Double) =
        targetPose?.let { state.Pose.translation.getDistance(it.translation) <= distance } == true

    private val alignmentDebouncer = Debouncer(0.05, Debouncer.DebounceType.kRising)

    private val posePIDController = PIDController(2.9, 0.0, 0.26) // 2.85, 0.0, 0.25 for 4piece auto

    /** Distance from goal pose when using PID driving */
    private var distanceFromPoseGoal = 0.0
    var hasPoseTarget = false
        private set

    /** Returns true if the robot is within distance when using PID drive to pose */
    private fun isPIDWithinGoal(distance: Double) = hasPoseTarget && distanceFromPoseGoal < distance

    /** Returns true if robot is with goal, either from PID or path planning driving */
    fun isWithinGoal(distance: Double) =
        isPIDWithinGoal(distance) || isPathPlannerWithinGoal(distance)

    /** Returns true if the robot is with its goal for a debounced period of time */
    fun isStableWithinGoal(distance: Double) = alignmentDebouncer.calculate(isWithinGoal(distance))

    /** Drives to a pose using a PID controller */
    fun driveToPose(pose: () -> Pose2d): Command =
        runOnce {
                val target = pose()
                val diff = target.minus(state.Pose)
                val distance = diff.translation.norm
                distanceFromPoseGoal = distance
                posePIDController.reset()

                Logger.recordOutput("DriveToPose target", target)
                FieldCentricFacingAngleAlignments.withTargetDirection(target.rotation)
                hasPoseTarget = true
            }
            .andThen(
                applyRequest {
                    val robot = Chassis.state.Pose
                    val target = pose()
                    val diff = robot.translation - target.translation

                    distanceFromPoseGoal = diff.norm
                    val output = posePIDController.calculate(diff.norm, 0.0)

                    val angle = diff.angle
                    val xSpeed = (output) * angle.cos
                    val ySpeed = (output) * angle.sin

                    FieldCentricFacingAngleAlignments.withVelocityX(xSpeed).withVelocityY(ySpeed)
                }
            )
            // Stop movement
            .finallyDo { _ ->
                setControl(ApplyRobotSpeeds())
                hasPoseTarget = false
            }

    private val pathConstraintZones =
        listOf(
            // Between approach point and final target, limit velocity to 0.5 meters per
            // second.
            // The other gains are arbitrarily high, since it likely doesn't matter what
            // those values are
            ConstraintsZone(1.0, 2.0, PathConstraints(0.5, 5.0, 10.0, 100.0, 12.0))
        )
    private val pathEventMarkers = listOf(EventMarker("atApproachPoint", 1.0))
    private val pathConstraints =
        PathConstraints(
            4.metersPerSecond,
            4.metersPerSecondPerSecond,
            1.67.rotationsPerSecond,
            3.rotationsPerSecondPerSecond,
            12.volts,
        )

    /**
     * Drives to a pose using pathplanner. The robot will always approach the target pose in the x
     * direction.
     *
     * @param approachBackward If true, the robot will approach the target pose in the -x direction.
     * @param pose The target pose to drive to.
     */
    fun pathplanToPose(
        approachBackward: () -> Boolean = { true },
        approachDistance: Distance = 0.5.meters,
        pose: () -> Pose2d,
    ): Command = defer {
        val currentPose = state.Pose
        val targetPose = pose()
        // Approach the target from 0.5 meter back
        val approachPoint =
            targetPose.transformBy(
                Transform2d(
                    // If approaching the target 'backward', approach from .5 meters in front of the
                    // robot, else, .5 meters behind the robot
                    if (approachBackward()) approachDistance else -approachDistance,
                    0.meters,
                    // If we're approaching the target 'backward', then the direction of travel
                    // should be 180º, i.e the back of the robot should be leading the path
                    if (approachBackward()) Rotation2d.k180deg else Rotation2d.kZero,
                )
            )
        val waypoints =
            PathPlannerPath.waypointsFromPoses(
                Pose2d(
                    currentPose.translation,
                    // Start the path in the direction of the approach point
                    (approachPoint.translation - currentPose.translation).angle,
                ),
                approachPoint,
                targetPose.transformBy(
                    Transform2d(
                        0.0,
                        0.0,
                        // If we're approaching the target 'backward', then the direction of travel
                        // should be 180º, i.e the back of the robot should be leading the path
                        if (approachBackward()) Rotation2d.k180deg else Rotation2d.kZero,
                    )
                ),
            )

        val path =
            PathPlannerPath(
                /* waypoints = */ waypoints,
                /* holonomicRotations = */ emptyList(),
                /* pointTowardsZones = */ emptyList(),
                /* constraintZones = */ pathConstraintZones,
                /* eventMarkers = */ pathEventMarkers,
                /* globalConstraints = */ pathConstraints,
                /* idealStartingState = */ null, // Unused in on-the-fly paths. Docs say use null
                /* goalEndState = */ GoalEndState(0.metersPerSecond, targetPose.rotation),
                /* reversed = */ false,
            )
        path.preventFlipping = true
        this.targetPose = targetPose
        AutoBuilder.followPath(path).finallyDo { _ -> this.targetPose = null }
    }

    // Drive to the center of the closest reef face, for picking up algae
    val driveToClosestReef by command {
        pathplanToPose {
            // Move into reef face by 2 inches more than default
            closestReef.transformBy(Transform2d((-2).inches, 0.inches, Rotation2d.kZero))
        }
    }

    val driveToClosestAlgae by command { driveToPose { closestAlgae } }

    /**
     * Returns a transform from the reef/branch to the robot depending on which level we're scoring
     * on and where the closest rotation point is. L1 and L2 can only score off the front, but L3
     * and L4 are free to score off the front or back
     */
    private fun bestReefToBotTransform(): Transform2d {
        return when (SelectedLevel) {
            L1 -> REEF_TO_ROBOT_FRONT_TRANSFORM
            L2,
            L3,
            L4 -> REEF_TO_ROBOT_BACK_TRANSFORM
        }
    }

    val driveToLeftBranch by command {
        pathplanToPoseWithCoralOffset { closestLeftBranch }.withName("Drive to branch left")
    }

    val driveToRightBranch by command {
        pathplanToPoseWithCoralOffset { closestRightBranch }.withName("Drive to branch left")
    }

    val driveToProcessor by command {
        pathplanToPose({ false }, approachDistance = .75.meters) { closestProcessor }
    }
    val backAwayFromProcessor by command {
        pathplanToPose(approachDistance = 0.meters) {
            closestProcessor.transformBy(Transform2d((-.5).meters, 0.meters, Rotation2d.kZero))
        }
    }

    val driveToClosestCenterCoralStation by command { driveToPose { closestCoralStation } }

    private val bargeApproachDistance = 0.5.meters
    val driveToBarge by command {
        pathplanToPose(approachDistance = bargeApproachDistance) { closestBarge }
    }
    val driveToBargeLeft by command {
        pathplanToPose(approachDistance = bargeApproachDistance) { closestLeftBarge }
    }
    val driveToBargeRight by command {
        pathplanToPose(approachDistance = bargeApproachDistance) { closestRightBarge }
    }

    val backAwayFromReef by command {
        pathplanToPose(approachDistance = 0.meters) {
            closestReef.transformBy(Transform2d(1.meters, 0.meters, Rotation2d.k180deg))
        }
    }

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
