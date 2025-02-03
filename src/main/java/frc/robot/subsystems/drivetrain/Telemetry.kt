package frc.robot.subsystems.drivetrain

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import frc.robot.lib.metersPerSecond

object Telemetry {
    private val MAX_SPEED = TunerConstants.kSpeedAt12Volts.metersPerSecond

    /* What to publish over networktables for telemetry */
    private val inst = NetworkTableInstance.getDefault()

    /* Robot swerve drive state */
    private val driveStateTable = inst.getTable("DriveState")
    private val drivePose: StructPublisher<Pose2d> =
        driveStateTable.getStructTopic("Pose", Pose2d.struct).publish()
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

    /* Mechanisms to represent the swerve module states */
    private val m_moduleMechanisms =
        arrayOf(
            Mechanism2d(1.0, 1.0),
            Mechanism2d(1.0, 1.0),
            Mechanism2d(1.0, 1.0),
            Mechanism2d(1.0, 1.0),
        )
    /* A direction and length changing ligament for speed representation */
    private val m_moduleSpeeds =
        arrayOf(
            m_moduleMechanisms[0]
                .getRoot("RootSpeed", 0.5, 0.5)
                .append(MechanismLigament2d("Speed", 0.5, 0.0)),
            m_moduleMechanisms[1]
                .getRoot("RootSpeed", 0.5, 0.5)
                .append(MechanismLigament2d("Speed", 0.5, 0.0)),
            m_moduleMechanisms[2]
                .getRoot("RootSpeed", 0.5, 0.5)
                .append(MechanismLigament2d("Speed", 0.5, 0.0)),
            m_moduleMechanisms[3]
                .getRoot("RootSpeed", 0.5, 0.5)
                .append(MechanismLigament2d("Speed", 0.5, 0.0)),
        )
    /* A direction changing and length constant ligament for module direction */
    private val m_moduleDirections =
        arrayOf(
            m_moduleMechanisms[0]
                .getRoot("RootDirection", 0.5, 0.5)
                .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
            m_moduleMechanisms[1]
                .getRoot("RootDirection", 0.5, 0.5)
                .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
            m_moduleMechanisms[2]
                .getRoot("RootDirection", 0.5, 0.5)
                .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
            m_moduleMechanisms[3]
                .getRoot("RootDirection", 0.5, 0.5)
                .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        )

    private val m_poseArray = DoubleArray(3)
    private val m_moduleStatesArray = DoubleArray(8)
    private val m_moduleTargetsArray = DoubleArray(8)

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

        /* Also write to log file */
        for (i in 0..6 step 2) {
            m_moduleStatesArray[i] = state.ModuleStates[i].angle.radians
            m_moduleStatesArray[i + 1] = state.ModuleStates[i].speedMetersPerSecond
            m_moduleTargetsArray[i] = state.ModuleTargets[i].angle.radians
            m_moduleTargetsArray[i + 1] = state.ModuleTargets[i].speedMetersPerSecond
        }

        SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray)
        SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray)
        SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray)
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds")

        /* Telemeterize the module states to a Mechanism2d */
        state.ModuleStates.forEachIndexed { i, moduleState ->
            m_moduleSpeeds[i].setAngle(moduleState.angle)
            m_moduleDirections[i].setAngle(moduleState.angle)
            m_moduleSpeeds[i].length = moduleState.speedMetersPerSecond / (2 * MAX_SPEED)
        }
        for (i in 0..3) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle)
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle)
            m_moduleSpeeds[i].length = state.ModuleStates[i].speedMetersPerSecond / (2 * MAX_SPEED)

            SmartDashboard.putData("Module $i", m_moduleMechanisms[i])
        }
    }
}
