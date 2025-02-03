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
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.subsystems.drivetrain.Chassis
import org.littletonrobotics.junction.Logger
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim

object Vision {
    // Measured from CAD
    private val camX = 9.486.inches
    private val camY = 10.309.inches
    private val camZ = 8.5.inches
    private val camRoll = 0.degrees
    private val camPitch = (-28.125).degrees
    private val camYaw = 30.degrees

    private val field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)

    private val cams: List<Camera> =
        listOf(
            Camera("FL", Transform3d(camX, camY, camZ, Rotation3d(camRoll, camPitch, camYaw))),
            Camera("FR", Transform3d(camX, -camY, camZ, Rotation3d(camRoll, camPitch, -camYaw))),
            Camera(
                "BL",
                Transform3d(-camX, camY, camZ, Rotation3d(camRoll, camPitch, 180.degrees - camYaw)),
            ),
            Camera(
                "BR",
                Transform3d(-camX, -camY, camZ, Rotation3d(camRoll, camPitch, 180.degrees + camYaw)),
            ),
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
                            Chassis.state.Pose +
                                Transform2d(
                                    cam.pose.translation.toTranslation2d(),
                                    cam.pose.rotation.toRotation2d(),
                                ) +
                                Transform2d(
                                    it.bestCameraToTarget.translation.toTranslation2d(),
                                    it.bestCameraToTarget.rotation.toRotation2d(),
                                )
                        }
                        .toTypedArray()
                )
            } // ?: kotlin.run {  if (cam.getTimeFromLastRefresh() > 1000.milliseconds)
            // Logger.recordOutput(cam.cam.name + " est", Pose2d())
            //            Logger.recordOutput(cam.cam.name + " tags", Translation3d())}
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

        cams.forEach { visionSim.addCamera(PhotonCameraSim(it.cam, camProps), it.pose) }

        Notifier { visionSim.update(Chassis.state.Pose) }.startPeriodic(0.020)
    }
}
