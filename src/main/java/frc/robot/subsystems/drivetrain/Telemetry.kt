package frc.robot.subsystems.drivetrain

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance

object Telemetry {

    /* What to publish over networktables for telemetry */
    private val inst = NetworkTableInstance.getDefault()

    /* Robot swerve drive state */
    private val driveStateTable = inst.getTable("DriveState")
    private val drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish()
    private val driveSpeeds =
        driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish()
    private val fieldRelativeSpeeds =
        driveStateTable.getStructTopic("FieldRelativeSpeeds", ChassisSpeeds.struct).publish()
    private val driveModuleStates =
        driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish()
    private val driveModuleTargets =
        driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish()
    private val driveModulePositions =
        driveStateTable
            .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
            .publish()
    private val driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish()
    private val driveOdometryFrequency =
        driveStateTable.getDoubleTopic("OdometryFrequency").publish()

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    fun telemeterize(state: SwerveDriveState) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose)
        driveSpeeds.set(state.Speeds)
        fieldRelativeSpeeds.set(
            ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.rotation)
        )
        driveModuleStates.set(state.ModuleStates)
        driveModuleTargets.set(state.ModuleTargets)
        driveModulePositions.set(state.ModulePositions)
        driveTimestamp.set(state.Timestamp)
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod)
    }
}
