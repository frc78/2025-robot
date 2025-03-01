package frc.robot.subsystems.drivetrain

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import org.littletonrobotics.junction.Logger

object Telemetry {

    /* What to publish over networktables for telemetry */
    private val inst = NetworkTableInstance.getDefault()

    /* Robot swerve drive state */
    private val driveStateTable = inst.getTable("DriveState")
    private val drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish()
    private val driveSpeeds =
        driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish()
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
        driveModuleStates.set(state.ModuleStates)
        driveModuleTargets.set(state.ModuleTargets)
        driveModulePositions.set(state.ModulePositions)
        driveTimestamp.set(state.Timestamp)
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod)

        Logger.recordOutput("DriveState/Pose", state.Pose)
        Logger.recordOutput(
            "DriveState/ModuleStates",
            SwerveModuleState.struct,
            *state.ModuleStates,
        )
        Logger.recordOutput(
            "DriveState/ModuleTargets",
            SwerveModuleState.struct,
            *state.ModuleTargets,
        )
        Logger.recordOutput("DriveState/OdometryPeriod", state.OdometryPeriod)
    }
}
