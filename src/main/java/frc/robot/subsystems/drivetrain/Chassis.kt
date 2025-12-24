package frc.robot.subsystems.drivetrain

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import frc.robot.generated.CompBotTunerConstants.*
import frc.robot.lib.FieldPoses.closestBarge
import frc.robot.lib.FieldPoses.closestBranch
import frc.robot.lib.FieldPoses.closestCoralStation
import frc.robot.lib.FieldPoses.closestLeftBarge
import frc.robot.lib.FieldPoses.closestProcessor
import frc.robot.lib.FieldPoses.closestReef
import frc.robot.lib.FieldPoses.closestRightBarge
import frc.robot.lib.bindings.ReefscapeController
import frc.robot.lib.inches
import frc.robot.lib.meters
import frc.robot.subsystems.Intake
import frc.robot.subsystems.LEDSubsystem
import frc.robot.subsystems.drivetrain.Chassis.ChassisState.AutoAlignAlgae
import frc.robot.subsystems.drivetrain.Chassis.ChassisState.AutoAlignCoral
import org.littletonrobotics.junction.Logger

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
@Suppress("UnusedPrivateProperty", "TooManyFunctions")
object Chassis :
    CompBotTunerSwerveDrivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight) {

    enum class ChassisState {
        FieldCentric,
        RobotCentric,
        AutoAlignCoral,
        AutoAlignAlgae,
    }

    private val table = NetworkTableInstance.getDefault().getTable("drivetrain")
    private val closestReefPub = table.getStructTopic("closest_reef", Pose2d.struct).publish()
    private val closestBranchPub = table.getStructTopic("closest_branch", Pose2d.struct).publish()
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
        if (Utils.isSimulation()) {
            startSimThread()
        }
    }

    var state_ = ChassisState.FieldCentric

    val coralAutoAlignDebouncer = Debouncer(0.3)
    val algaeAutoAlignDebounce = Debouncer(0.3)

    fun stateMachine() {
        when (state_) {
            ChassisState.FieldCentric -> {
                setControl(
                    FieldCentric.withVelocityX(ReefscapeController.velocityX)
                        .withVelocityY(ReefscapeController.velocityY)
                        .withRotationalRate(ReefscapeController.velocityRot)
                )
                if (ReefscapeController.robotCentric()) {
                    state_ = ChassisState.RobotCentric
                }
            }
            ChassisState.RobotCentric -> {
                setControl(
                    RobotCentric.withVelocityX(ReefscapeController.velocityX)
                        .withVelocityY(ReefscapeController.velocityY)
                        .withRotationalRate(ReefscapeController.velocityRot)
                )
                if (ReefscapeController.fieldCentric()) {
                    state_ = ChassisState.FieldCentric
                }
            }
            AutoAlignCoral -> {
                if (driveToPoseWithCoralOffset(closestBranch)) {
                    LEDSubsystem.state = LEDSubsystem.LedState.Aligned
                }
                if (
                    !(ReefscapeController.l2() ||
                        ReefscapeController.l3() ||
                        ReefscapeController.l4())
                ) {
                    LEDSubsystem.state = LEDSubsystem.LedState.Idle
                    state_ = ChassisState.FieldCentric
                }
            }

            AutoAlignAlgae -> {
                driveToPose(closestReef, 0.0)
                if (!(ReefscapeController.highAlgae() || ReefscapeController.lowAlgae())) {
                    state_ = ChassisState.FieldCentric
                }
            }
        }
        if (
            coralAutoAlignDebouncer.calculate(
                Intake.holdingCoral &&
                    (ReefscapeController.l2() ||
                        ReefscapeController.l3() ||
                        ReefscapeController.l4())
            )
        ) {
            posePIDController.reset()
            coralAutoAlignDebouncer.calculate(false)
            state_ = AutoAlignCoral
        } else if (
            algaeAutoAlignDebounce.calculate(
                !(Intake.holdingAlgae || Intake.holdingCoral) &&
                    (ReefscapeController.highAlgae() || ReefscapeController.lowAlgae())
            )
        ) {
            posePIDController.reset()
            coralAutoAlignDebouncer.calculate(false)
            state_ = AutoAlignAlgae
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

    private val RobotCentric =
        SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

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

    fun periodic() {
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
        Logger.recordOutput("chassis/state", state_.name)
    }

    /** Drives to a pose such that the coral is at x=0 */
    fun driveToPoseWithCoralOffset(pose: Pose2d) =
        driveToPose(
            pose.transformBy(Transform2d(0.inches, -Intake.coralLocation, Rotation2d.kZero)),
            1.inches.meters,
        )

    private val posePIDController = PIDController(2.9, 0.0, 0.26) // 2.85, 0.0, 0.25 for 4piece auto

    /**
     * Drives to a pose using a PID controller
     *
     * @param threshold distance from target to consider "at target", in meters
     * @return true if distance to target is < threshold
     */
    fun driveToPose(target: Pose2d, threshold: Double): Boolean {
        Logger.recordOutput("DriveToPose target", target)
        val diff = state.Pose.translation - target.translation
        FieldCentricFacingAngleAlignments.withTargetDirection(target.rotation)

        val output = posePIDController.calculate(diff.norm)

        val angle = diff.angle
        val xSpeed = (output) * angle.cos
        val ySpeed = (output) * angle.sin

        setControl(FieldCentricFacingAngleAlignments.withVelocityX(xSpeed).withVelocityY(ySpeed))
        return diff.norm < threshold
    }
}
