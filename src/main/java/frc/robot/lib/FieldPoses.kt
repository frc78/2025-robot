package frc.robot.lib

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import frc.robot.Robot
import frc.robot.Robot.alliance
import frc.robot.subsystems.drivetrain.Chassis
import kotlin.jvm.optionals.getOrNull

/** Poses that the robot can auto-align to */
object FieldPoses {
    val REEF_TO_BRANCH_LEFT = Transform2d(0.meters, -(13 / 2).inches, Rotation2d.kZero)
    val REEF_TO_BRANCH_RIGHT = Transform2d(0.meters, (13 / 2).inches, Rotation2d.kZero)
    private val REEF_TO_BOT_TRANSFORM = Transform2d(0.72.meters, 0.meters, Rotation2d.kZero)
    private val CORAL_TO_BOT_TRANSFORM = Transform2d(.5.meters, 0.meters, Rotation2d.k180deg)

    // Tag IDs are in order of ReefFaces
    private val BLUE_REEF_POSES =
        intArrayOf(18, 17, 22, 21, 20, 19).map {
            Robot.gameField.getTagPose(it).get().toPose2d().transformBy(REEF_TO_BOT_TRANSFORM)
        }
    private val BLUE_BRANCH_POSES =
        BLUE_REEF_POSES.flatMap {
            listOf(it.transformBy(REEF_TO_BRANCH_LEFT), it.transformBy(REEF_TO_BRANCH_RIGHT))
        }

    // Tag IDs are in order of ReefFaces
    private val RED_REEF_POSES =
        intArrayOf(7, 8, 9, 10, 11, 6).map {
            Robot.gameField.getTagPose(it).get().toPose2d().transformBy(REEF_TO_BOT_TRANSFORM)
        }

    private val RED_BRANCH_POSES =
        RED_REEF_POSES.flatMap {
            listOf(it.transformBy(REEF_TO_BRANCH_LEFT), it.transformBy(REEF_TO_BRANCH_RIGHT))
        }

    private val HIGH_ALGAE_REEF_POSES =
        listOf(
            BLUE_REEF_POSES[0],
            BLUE_REEF_POSES[2],
            BLUE_REEF_POSES[4],
            RED_REEF_POSES[0],
            RED_REEF_POSES[2],
            RED_REEF_POSES[4],
        )

    val closestAlgaeIsHigh
        get() = closestReef in HIGH_ALGAE_REEF_POSES

    private val CORAL_STATION_LEFT = Transform2d(0.inches, 24.inches, Rotation2d.kZero)
    private val CORAL_STATION_RIGHT = Transform2d(0.inches, (-24).inches, Rotation2d.kZero)

    private val BLUE_CORAL_STATION_LOCATIONS =
        intArrayOf(12, 13).map {
            Robot.gameField.getTagPose(it).get().toPose2d().transformBy(CORAL_TO_BOT_TRANSFORM)
        }
    private val RED_CORAL_STATION_LOCATIONS =
        intArrayOf(1, 2).map {
            Robot.gameField.getTagPose(it).get().toPose2d().transformBy(CORAL_TO_BOT_TRANSFORM)
        }

    private val reefPoses = BLUE_REEF_POSES + RED_REEF_POSES

    private val allianceReefPoses
        get() =
            if (Robot.alliance == Alliance.Blue) {
                BLUE_REEF_POSES
            } else RED_REEF_POSES

    private val branchPoses
        get() =
            if (Robot.alliance == Alliance.Blue) {
                BLUE_BRANCH_POSES
            } else RED_BRANCH_POSES

    val closestReef: Pose2d
        get() = Chassis.state.Pose.nearest(reefPoses)

    val closestBranch: Pose2d
        get() = Chassis.state.Pose.nearest(branchPoses)

    /** Returns true if the pose is on the far side of the reef from the alliance wall */
    val Pose2d.isFarReef: Boolean
        get() {
            val myReefs = allianceReefPoses
            return myReefs[2] == this || myReefs[3] == this || myReefs[4] == this
        }

    val closestLeftBranch: Pose2d
        get() {
            closestReef.let {
                // reverse left/right branches for EF, GH, and IJ
                return if (it.isFarReef) {
                    it.transformBy(REEF_TO_BRANCH_RIGHT)
                } else it.transformBy(REEF_TO_BRANCH_LEFT)
            }
        }

    val closestRightBranch: Pose2d
        get() {
            closestReef.let {
                return if (it.isFarReef) {
                    it.transformBy(REEF_TO_BRANCH_LEFT)
                } else it.transformBy(REEF_TO_BRANCH_RIGHT)
            }
        }

    val closestCoralStation: Pose2d
        get() {
            val alliance = DriverStation.getAlliance().getOrNull() ?: return Pose2d.kZero
            val allianceCoralStations =
                when (alliance) {
                    Red -> RED_CORAL_STATION_LOCATIONS
                    Blue -> BLUE_CORAL_STATION_LOCATIONS
                }
            return Chassis.state.Pose.nearest(allianceCoralStations)
        }

    val closestLeftCoralStation
        get() = closestCoralStation.transformBy(CORAL_STATION_LEFT)

    val closestRightCoralStation
        get() = closestCoralStation.transformBy(CORAL_STATION_RIGHT)

    enum class ReefFace {
        AB,
        CD,
        EF,
        GH,
        IJ,
        KL;

        val pose
            get() = reefPoses[ordinal]

        val leftBranch
            get() = branchPoses[ordinal * 2]

        val rightBranch
            get() = branchPoses[ordinal * 2 + 1]
    }

    enum class Branch(val reefFace: ReefFace, val left: Boolean) {
        A(ReefFace.AB, true),
        B(ReefFace.AB, false),
        C(ReefFace.CD, true),
        D(ReefFace.CD, false),
        E(ReefFace.EF, true),
        F(ReefFace.EF, false),
        G(ReefFace.GH, true),
        H(ReefFace.GH, false),
        I(ReefFace.IJ, true),
        J(ReefFace.IJ, false),
        K(ReefFace.KL, true),
        L(ReefFace.KL, false);

        val pose
            get() = if (left) reefFace.leftBranch else reefFace.rightBranch
    }

    private val PROCESSOR_TO_BOT = Transform2d(0.9.meters, 0.0.meters, Rotation2d.k180deg)

    private val PROCESSOR_POSES =
        intArrayOf(3, 16).map {
            Robot.gameField.getTagPose(it).get().toPose2d().transformBy(PROCESSOR_TO_BOT)
        }

    val closestProcessor: Pose2d
        get() = Chassis.state.Pose.nearest(PROCESSOR_POSES)

    // Offset from the april tag coordinate to where the robot needs to be to score/align
    private val BARGE_TO_BOT = Transform2d(0.5922.meters, 0.0.meters, Rotation2d.kZero)

    private val BLUE_BARGE_POSES =
        intArrayOf(4, 14).map {
            Robot.gameField.getTagPose(it).get().toPose2d().transformBy(BARGE_TO_BOT)
        }

    private val RED_BARGE_POSES =
        intArrayOf(5, 15).map {
            Robot.gameField.getTagPose(it).get().toPose2d().transformBy(BARGE_TO_BOT)
        }

    private val bargePoses
        get() =
            when (alliance) {
                Blue -> BLUE_BARGE_POSES
                Red -> RED_BARGE_POSES
            }

    private val BARGE_TO_BARGE_LEFT = Transform2d(0.meters, (-42.875).inches, Rotation2d.kZero)
    private val BARGE_TO_BARGE_RIGHT = Transform2d(0.meters, 42.875.inches, Rotation2d.kZero)

    val closestBarge
        get() = Chassis.state.Pose.nearest(bargePoses)

    val closestLeftBarge
        get() = closestBarge.transformBy(BARGE_TO_BARGE_LEFT)

    val closestRightBarge
        get() = closestBarge.transformBy(BARGE_TO_BARGE_RIGHT)
}
