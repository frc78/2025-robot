package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Notifier
import frc.robot.IS_TEST
import frc.robot.lib.centimeters
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.subsystems.drivetrain.Chassis
import org.littletonrobotics.junction.Logger
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim

object Vision {

    private val field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)

    private val camPitch = (-14).degrees
    private val camRoll = 0.degrees
    private val camX = (51.5 / 2).centimeters
    private val camZ = 8.5.inches

    private val cams: List<Camera> =
        listOfNotNull(
            Camera(
                    "FL",
                    Transform3d(
                        camX,
                        (50 / 2).centimeters,
                        camZ,
                        Rotation3d(camRoll, camPitch, (-28.579 - 270).degrees),
                    ),
                )
                .takeIf { IS_TEST },
            Camera(
                    "FR",
                    Transform3d(
                        camX,
                        (-50 / 2).centimeters,
                        camZ,
                        Rotation3d(0.degrees, camPitch, (-28.579).degrees),
                    ),
                )
                .takeIf { IS_TEST },
            Camera(
                    "BL",
                    Transform3d(
                        -camX,
                        (55 / 2).centimeters,
                        camZ,
                        Rotation3d(0.degrees, camPitch, 225.degrees),
                    ),
                )
                .takeIf { IS_TEST },
            Camera(
                    "BR",
                    Transform3d(
                        -camX,
                        (-55 / 2).centimeters,
                        camZ,
                        Rotation3d(0.degrees, camPitch, 135.degrees),
                    ),
                )
                .takeIf { IS_TEST },
            Camera("PLACEHOLDER", Transform3d()).takeIf { !IS_TEST },
        )

    private val table = NetworkTableInstance.getDefault().getTable("vision")
    private val topics =
        cams.associateWith { table.getStructArrayTopic("${it.name} tags", Pose2d.struct).publish() }

    fun update() {
        cams.forEach { cam ->
            cam.getEstimatedGlobalPose()?.let {
                val pose = it.estimatedPose.toPose2d()
                Chassis.addVisionMeasurement(pose, it.timestampSeconds, cam.currentStds)
                Logger.recordOutput(cam.cam.name + " est", pose)
                // Send the locations of all tracked tags to networktables
                topics[cam]?.set(
                    it.targetsUsed
                        .map {
                            (Chassis.state.Pose + cam.robotToCamera2d) +
                                Transform2d(
                                    it.bestCameraToTarget.translation.toTranslation2d(),
                                    it.bestCameraToTarget.rotation.toRotation2d(),
                                )
                        }
                        .toTypedArray()
                )
            }
                ?: run {
                    Logger.recordOutput(cam.cam.name + " est", Pose2d())
                    topics[cam]?.set(emptyArray())
                }
        }
    }

    /** Initializes simulated cameras and runs an update thread at 50 Hz */
    fun setupSimulation() {
        val visionSim = VisionSystemSim("sim")
        visionSim.addAprilTags(field)

        val camProps =
            SimCameraProperties().apply {
                setCalibration(1200, 720, Rotation2d.fromDegrees(70.0))
                fps = 25.0
            }

        cams.forEach { visionSim.addCamera(PhotonCameraSim(it.cam, camProps), it.transform) }

        Notifier { visionSim.update(Chassis.state.Pose) }.startPeriodic(0.020)
    }
}
