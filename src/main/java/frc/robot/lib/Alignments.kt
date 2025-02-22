package frc.robot.lib

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.subsystems.drivetrain.Chassis
import kotlin.jvm.optionals.getOrNull

/** Poses that the robot can auto-align to */
object Alignments {
    // This field will need to be updated based on the event venue.
    private val aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)

    val REEF_TO_BRANCH_LEFT = Transform2d(0.meters, -(13 / 2).inches, Rotation2d.kZero)
    val REEF_TO_BRANCH_RIGHT = Transform2d(0.meters, (13 / 2).inches, Rotation2d.kZero)
    private val REEF_TO_BOT_TRANSFORM = Transform2d(0.4.meters, 0.meters, Rotation2d.kZero)

    private val BLUE_REEF_POSES =
        intArrayOf(17, 18, 19, 20, 21, 22).map {
            aprilTags.getTagPose(it).get().toPose2d().transformBy(REEF_TO_BOT_TRANSFORM)
        }
    private val BLUE_BRANCH_POSES =
        BLUE_REEF_POSES.flatMap {
            listOf(it.transformBy(REEF_TO_BRANCH_LEFT), it.transformBy(REEF_TO_BRANCH_RIGHT))
        }

    private val RED_REEF_POSES =
        intArrayOf(6, 7, 8, 9, 10, 11).map { aprilTags.getTagPose(it).get().toPose2d() }

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
        intArrayOf(12, 13).flatMap { aprilTags.getTagPose(it).get().toPose2d().coralPosesFromTag }
    private val RED_CORAL_STATION_LOCATIONS =
        intArrayOf(1, 2).flatMap { aprilTags.getTagPose(it).get().toPose2d().coralPosesFromTag }

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
}
