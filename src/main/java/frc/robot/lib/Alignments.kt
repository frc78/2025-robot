package frc.robot.lib

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.Robot
import frc.robot.subsystems.drivetrain.Chassis
import kotlin.jvm.optionals.getOrNull

/** Poses that the robot can auto-align to */
object Alignments {
    val REEF_TO_BRANCH_LEFT = Transform2d(0.meters, -(13 / 2).inches, Rotation2d.kZero)
    val REEF_TO_BRANCH_RIGHT = Transform2d(0.meters, (13 / 2).inches, Rotation2d.kZero)
    private val REEF_TO_BOT_TRANSFORM = Transform2d(0.7.meters, 0.meters, Rotation2d.kZero)
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

    private val CORAL_STATION_LEFT = Transform2d(0.inches, 24.inches, Rotation2d.kZero)
    private val CORAL_STATION_RIGHT = Transform2d(0.inches, (-24).inches, Rotation2d.kZero)

    private val Pose2d.coralPosesFromTag
        get() =
            listOf(
                this,
                this.transformBy(CORAL_STATION_LEFT),
                this.transformBy(CORAL_STATION_RIGHT),
            )

    private val BLUE_CORAL_STATION_LOCATIONS =
        intArrayOf(12, 13).flatMap {
            Robot.gameField.getTagPose(it).get().toPose2d().coralPosesFromTag.map {
                it.transformBy(CORAL_TO_BOT_TRANSFORM)
            }
        }
    private val RED_CORAL_STATION_LOCATIONS =
        intArrayOf(1, 2).flatMap {
            Robot.gameField.getTagPose(it).get().toPose2d().coralPosesFromTag.map {
                it.transformBy(CORAL_TO_BOT_TRANSFORM)
            }
        }

    private val reefPoses
        get() =
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) BLUE_REEF_POSES
            else RED_REEF_POSES

    private val branchPoses
        get() =
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                BLUE_BRANCH_POSES
            } else RED_BRANCH_POSES

    val closestReef: Pose2d
        get() = Chassis.state.Pose.nearest(reefPoses)

    val closestBranch: Pose2d
        get() = Chassis.state.Pose.nearest(branchPoses)

    val closestLeftBranch: Pose2d
        get() = closestBranch.transformBy(REEF_TO_BRANCH_LEFT)

    val closestRightBranch: Pose2d
        get() = closestBranch.transformBy(REEF_TO_BRANCH_RIGHT)

    val closestCoralStation: Pose2d
        get() {
            val alliance = DriverStation.getAlliance().getOrNull() ?: return Pose2d.kZero
            val allianceCoralStations =
                when (alliance) {
                    Alliance.Red -> RED_CORAL_STATION_LOCATIONS
                    Alliance.Blue -> BLUE_CORAL_STATION_LOCATIONS
                }
            return Chassis.state.Pose.nearest(allianceCoralStations)
        }

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
}
