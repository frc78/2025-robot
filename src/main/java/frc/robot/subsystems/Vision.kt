package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Notifier
import frc.robot.IS_TEST
import frc.robot.Robot
import frc.robot.lib.centimeters
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.subsystems.drivetrain.Chassis
import org.littletonrobotics.junction.Logger
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim

object Vision {
    private val testCamPitch = (-14).degrees
    private val testCamRoll = 0.degrees
    private val testCamX = (51.5 / 2).centimeters
    private val testCamZ = 8.5.inches

    private val cams: List<Camera> =
        listOfNotNull(
            Camera(
                    "FL",
                    Transform3d(
                        testCamX,
                        (50 / 2).centimeters,
                        testCamZ,
                        Rotation3d(testCamRoll, testCamPitch, (-28.579 - 270).degrees),
                    ),
                )
                .takeIf { IS_TEST },
            Camera(
                    "FR",
                    Transform3d(
                        testCamX,
                        (-50 / 2).centimeters,
                        testCamZ,
                        Rotation3d(0.degrees, testCamPitch, (-28.579).degrees),
                    ),
                )
                .takeIf { IS_TEST },
            Camera(
                    "BL",
                    Transform3d(
                        -testCamX,
                        (55 / 2).centimeters,
                        testCamZ,
                        Rotation3d(0.degrees, testCamPitch, 225.degrees),
                    ),
                )
                .takeIf { IS_TEST },
            Camera(
                    "BR",
                    Transform3d(
                        -testCamX,
                        (-55 / 2).centimeters,
                        testCamZ,
                        Rotation3d(0.degrees, testCamPitch, 135.degrees),
                    ),
                )
                .takeIf { IS_TEST },
            Camera(
                    "BowCam",
                    Transform3d(
                        11.830.inches,
                        (-11.491).inches,
                        8.503.inches,
                        Rotation3d(0.degrees, (-14).degrees, 28.597.degrees),
                    ),
                )
                .takeIf { !IS_TEST },
            Camera(
                    "PortCam",
                    Transform3d(
                        (-11.857).inches,
                        (11.483).inches,
                        8.503.inches,
                        Rotation3d(0.degrees, (-14).degrees, 200.degrees),
                    ),
                )
                .takeIf { !IS_TEST },
            Camera(
                    "StarboardCam",
                    Transform3d(
                        (-11.857).inches,
                        (-11.483).inches,
                        8.503.inches,
                        Rotation3d(0.degrees, (-14).degrees, (-200).degrees),
                    ),
                )
                .takeIf { !IS_TEST },
        )

    private val table = NetworkTableInstance.getDefault().getTable("vision")
    private val topics =
        cams.associateWith { table.getStructArrayTopic("${it.name} tags", Pose2d.struct).publish() }

    fun update() {
        cams.forEach { cam ->
            cam.getEstimatedGlobalPose()?.let { estimatedGlobalPose ->
                val pose = estimatedGlobalPose.estimatedPose.toPose2d()
                Chassis.addVisionMeasurement(
                    pose,
                    estimatedGlobalPose.timestampSeconds,
                    cam.currentStds,
                )
                Logger.recordOutput(cam.cam.name + " est", pose)
                // Send the locations of all tracked tags to networktables
                topics[cam]?.set(
                    estimatedGlobalPose.targetsUsed
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
            //                ?: run {
            //                    Logger.recordOutput(cam.cam.name + " est", Pose2d())
            //                    topics[cam]?.set(emptyArray())
            //                }
        }
    }

    /** Initializes simulated cameras and runs an update thread at 50 Hz */
    fun setupSimulation() {
        val visionSim = VisionSystemSim("sim")
        visionSim.addAprilTags(Robot.gameField)

        val camProps =
            SimCameraProperties().apply {
                setCalibration(1200, 720, Rotation2d.fromDegrees(70.0))
                fps = 25.0
            }

        cams.forEach { visionSim.addCamera(PhotonCameraSim(it.cam, camProps), it.transform) }

        Notifier { visionSim.update(Chassis.state.Pose) }.startPeriodic(0.020)
    }
}
