package frc.robot.subsystems

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.bindings.ReefscapeController
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.subsystems.Intake.IntakeState
import frc.robot.subsystems.Intake.IntakeState.HoldCoral
import frc.robot.subsystems.SuperStructure.SuperStructureState.*
import org.littletonrobotics.junction.Logger

object SuperStructure {

    /** @property pivotAngle: Angle of the pivot from horizontal */
    enum class SuperStructureState(
        val pivotAngle: Angle,
        val elevatorHeight: Distance,
        val wristAngle: Angle,
    ) {
        Home(40.degrees, 0.25.inches, 22.5.degrees),
        L1(38.degrees, 0.25.inches, 186.75.degrees),
        L2(93.5.degrees, 0.25.inches, 39.degrees),
        L3(92.6.degrees, 17.25.inches, 38.degrees),
        L4(90.5.degrees, 47.inches, 29.25.degrees), // 92.5 pivot
        CoralStation(66.85.degrees, 0.1.inches, 185.8.degrees), // pivot 68.2 // 67.1 pivot 6/8
        FloorAlgae(26.degrees, 0.25.inches, 181.35.degrees),
        Processor(10.7.degrees, 0.25.inches, 86.4.degrees),
        HighAlgae(100.2.degrees, 19.33.inches, 13.25.degrees),
        LowAlgae(104.degrees, 0.25.inches, 19.2125.degrees),
        Net(91.degrees, 53.5.inches, 47.7675.degrees),
        ReadyToClimb(70.degrees, 0.25.inches, 180.degrees),
        FullyClimbed(5.degrees, 0.25.inches, 90.degrees),
    }

    var currentState = Home

    fun stateMachine() {
        val intakeIdle = Intake.currentState == IntakeState.Idle
        val intakeHasCoral = Intake.currentState == HoldCoral
        val intakeHasAlgae = Intake.currentState == IntakeState.HoldAlgae
        // If moving to a lower elevator position, move elevator first
        if (currentState.elevatorHeight < Elevator.position) {
            Elevator.goTo(currentState.elevatorHeight)
            if ((Elevator.position - currentState.elevatorHeight).abs(Inches) < 0.5) {
                Wrist.goTo(currentState.wristAngle)
                Pivot.goTo(currentState.pivotAngle)
            }
        } else {
            // Move pivot and wrist, then elevator
            Pivot.goTo(currentState.pivotAngle)
            Wrist.goTo(currentState.wristAngle)
            if ((Pivot.angle - currentState.pivotAngle).abs(Degrees) < 2.0) {
                Elevator.goTo(currentState.elevatorHeight)
            }
        }
        when (currentState) {
            Home -> {
                if (intakeHasCoral) {
                    if (ReefscapeController.l1()) {
                        currentState = L1
                    } else if (ReefscapeController.l2()) {
                        currentState = L2
                    } else if (ReefscapeController.l3()) {
                        currentState = L3
                    } else if (ReefscapeController.l4()) {
                        currentState = L4
                    }
                } else if (intakeHasAlgae) {
                    if (ReefscapeController.net()) {
                        currentState = Net
                    } else if (ReefscapeController.process()) {
                        currentState = Processor
                    }
                } else {
                    if (ReefscapeController.coral()) {
                        currentState = CoralStation
                    } else if (ReefscapeController.floorAlgae()) {
                        currentState = FloorAlgae
                    } else if (ReefscapeController.highAlgae()) {
                        currentState = HighAlgae
                    } else if (ReefscapeController.lowAlgae()) {
                        currentState = LowAlgae
                    }
                }
                if (ReefscapeController.prepareClimb()) {
                    currentState = ReadyToClimb
                }
            }
            L1 -> {
                if (ReefscapeController.l2()) {
                    currentState = L2
                } else if (ReefscapeController.l3()) {
                    currentState = L3
                } else if (ReefscapeController.l4()) {
                    currentState = L4
                } else if (ReefscapeController.home()) {
                    currentState = Home
                }
            }
            L2 -> {
                if (ReefscapeController.l1()) {
                    currentState = L1
                } else if (ReefscapeController.l3()) {
                    currentState = L3
                } else if (ReefscapeController.l4()) {
                    currentState = L4
                } else if (ReefscapeController.home()) {
                    currentState = Home
                }
            }
            L3 -> {
                if (ReefscapeController.l1()) {
                    currentState = L1
                } else if (ReefscapeController.l2()) {
                    currentState = L2
                } else if (ReefscapeController.l4()) {
                    currentState = L4
                } else if (ReefscapeController.home()) {
                    currentState = Home
                }
            }
            L4 -> {
                if (ReefscapeController.l1()) {
                    currentState = L1
                } else if (ReefscapeController.l2()) {
                    currentState = L2
                } else if (ReefscapeController.l3()) {
                    currentState = L3
                } else if (ReefscapeController.home()) {
                    currentState = Home
                }
            }
            CoralStation -> {
                if (ReefscapeController.home()) {
                    currentState = Home
                }
            }
            FloorAlgae -> {
                if (ReefscapeController.home()) {
                    currentState = Home
                }
            }
            Processor -> {
                if (ReefscapeController.home()) {
                    currentState = Home
                }
            }
            HighAlgae -> {
                if (ReefscapeController.home()) {
                    currentState = Home
                } else if (ReefscapeController.lowAlgae()) {
                    currentState = LowAlgae
                }
            }
            LowAlgae -> {
                if (ReefscapeController.home()) {
                    currentState = Home
                } else if (ReefscapeController.highAlgae()) {
                    currentState = HighAlgae
                }
            }
            Net -> {
                if (ReefscapeController.home()) {
                    currentState = Home
                }
            }
            ReadyToClimb -> {
                if (ReefscapeController.climb()) {
                    currentState = FullyClimbed
                } else if (ReefscapeController.home()) {
                    currentState = Home
                }
            }
            FullyClimbed -> Unit
        }
    }

    fun periodic() {
        Elevator.periodic()
        Pivot.periodic()
        Wrist.periodic()
        Logger.recordOutput("superstructure/state", currentState.name)
    }

    fun simulationPeriodic() {
        Elevator.simulationPeriodic()
        Pivot.simulationPeriodic()
        Wrist.simulationPeriodic()
    }
}
