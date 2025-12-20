// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj.util.WPILibVersion
import frc.robot.auto.OPAuto
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.lib.meters
import frc.robot.subsystems.*
import frc.robot.subsystems.drivetrain.Chassis
import frc.robot.subsystems.drivetrain.Telemetry
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.photonvision.PhotonPoseEstimator

object Robot : LoggedRobot() {
    val gameField: AprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
    val reefOnlyField: AprilTagFieldLayout =
        AprilTagFieldLayout(Filesystem.getDeployDirectory().path + "/2025-welded-only-reef.json")
    val alliance: DriverStation.Alliance
        get() = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)

    init {
        gameField.tags
        HAL.report(
            FRCNetComm.tResourceType.kResourceType_Language,
            FRCNetComm.tInstances.kLanguage_Kotlin,
            0,
            WPILibVersion.Version,
        )
        DriverStation.silenceJoystickConnectionWarning(true)
        // Publish data to NetworkTables
        Logger.addDataReceiver(NT4Publisher())
        PowerDistribution(1, PowerDistribution.ModuleType.kRev)
        Logger.start()
        if (isReal()) {
            // DataLog will automatically log all NT changes. AKit logs to NT, DataLog logs NT to
            // file
            DataLogManager.start()
            SignalLogger.setPath("/U/ctre-logs")
            SignalLogger.start()
        }
        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog())
        Chassis.registerTelemetry(Telemetry::telemeterize)

        Pivot.coast()
        StateMachineManager.robotInit()
    }

    /* lateinit is a way to tell the compiler that we promise to initialize this variable before
    using them. These are lateinit since we don't want to create them always, but when we access them in
    simulationPeriodic we can be confident they were created in simulationInit */
    private lateinit var elevatorMech: MechanismLigament2d
    private lateinit var wristMech: MechanismLigament2d

    override fun simulationInit() {
        StateMachineManager.simulationInit()
        // Create a mechanism widget
        val robot = Mechanism2d(1.0, 5.0)
        // The pivot axis location is the root of the mechanism
        // This puts it at 5 units from the left and 5 units from the bottom of the widget
        val pivot = robot.getRoot("pivot", .25, 0.25)
        // The elevator gets attached to the pivot. It starts with a length of 30, and an angle of 4
        // degrees
        elevatorMech =
            pivot.append(
                MechanismLigament2d(
                    "elevator",
                    30.0.inches.meters,
                    45.0,
                    4.0,
                    Color8Bit(Color.kOrange),
                )
            )
        // The wrist gets attached to the elevator. It starts with a length of 20, and an angle of
        // 90.
        // This angle is relative to the elevator, so it will point straight up when the elevator is
        // at 0º, and will point
        // to the left when the elevator is at 90º
        wristMech =
            elevatorMech.append(
                MechanismLigament2d(
                    "wrist",
                    12.0.inches.meters,
                    90.0,
                    6.0,
                    Color8Bit(Color.kPurple),
                )
            )
        wristMech.append(
            MechanismLigament2d("coral", 6.25.inches.meters, 112.0, 6.0, Color8Bit(Color.kWhite))
        )

        // Put the mechanism widget with all its components on the dashboard,
        SmartDashboard.putData("robot", robot)
        //        Vision.setupSimulation()
    }

    override fun robotPeriodic() {
        StateMachineManager.robotPeriodic()
        if (isReal()) {
            Vision.update()
        }
    }

    override fun autonomousPeriodic() {
        OPAuto.runAuto()
        StateMachineManager.autonomousPeriodic()
    }

    override fun teleopPeriodic() {
        StateMachineManager.teleopPeriodic()
    }

    override fun simulationPeriodic() {
        StateMachineManager.simulationPeriodic()

        // The angle of the elevator is determined by the pivot angle
        // The angle parameter expects a value in degrees, so we convert it to degrees
        elevatorMech.angle = Pivot.angle.degrees
        // Elevator.length is 0 when the elevator is retracted, but the elevator has a fixed length
        // of 30 inches
        elevatorMech.length = (30.inches + Elevator.position).meters
        wristMech.angle = 140 - Wrist.angle.degrees
    }

    override fun autonomousInit() {
        StateMachineManager.autonomousInit()
        OPAuto.init()
    }

    override fun disabledInit() {
        StateMachineManager.disabledInit()
        Vision.setMultitagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
    }

    private var pivotBraked = false

    override fun disabledPeriodic() {
        StateMachineManager.disabledPeriodic()
        if (Pivot.angle > 45.degrees && !pivotBraked) {
            Pivot.brake()
            pivotBraked = true
        }
    }

    override fun disabledExit() {
        Vision.setMultitagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE)
    }
}
