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
        PreScoreL4(90.degrees, 0.25.inches, 29.25.degrees),
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

    val atPosition
        get() = Elevator.atPosition && Wrist.atPosition && Pivot.atPosition

    var state = Home

    fun stateMachine() {
        val intakeHasCoral = Intake.state == HoldCoral
        val intakeHasAlgae = Intake.state == IntakeState.HoldAlgae
        // If moving to a lower elevator position, move wrist after elevator is down
        if (state.elevatorHeight < Elevator.position) {
            Elevator.goTo(state.elevatorHeight)
            Pivot.goTo(state.pivotAngle)
            if ((Elevator.position - state.elevatorHeight).abs(Inches) < 0.5) {
                Wrist.goTo(state.wristAngle)
            }
        } else {
            // Move pivot and wrist, then elevator
            Pivot.goTo(state.pivotAngle)
            Wrist.goTo(state.wristAngle)
            if ((Pivot.angle - state.pivotAngle).abs(Degrees) < 2.0) {
                Elevator.goTo(state.elevatorHeight)
            }
        }
        when (state) {
            Home -> {
                if (!(intakeHasAlgae || intakeHasCoral)) {
                    if (ReefscapeController.coral()) {
                        state = CoralStation
                    } else if (ReefscapeController.floorAlgae()) {
                        state = FloorAlgae
                    } else if (ReefscapeController.highAlgae()) {
                        state = HighAlgae
                    } else if (ReefscapeController.lowAlgae()) {
                        state = LowAlgae
                    }
                } else if (intakeHasCoral) {
                    processCoralScoring()
                } else {
                    // intake necesarily has algae
                    if (ReefscapeController.process()) {
                        state = Processor
                    } else if (ReefscapeController.net()) {
                        state = Net
                    }
                }
                if (ReefscapeController.prepareClimb()) {
                    state = ReadyToClimb
                }
            }
            CoralStation -> {
                // Can go to coral scoring once holding coral
                if (ReefscapeController.home()) {
                    state = Home
                }
            }
            FloorAlgae -> {
                if (ReefscapeController.home()) {
                    state = Home
                }
            }
            Processor -> {
                if (ReefscapeController.home()) {
                    state = Home
                }
            }
            HighAlgae -> {
                if (ReefscapeController.home()) {
                    state = Home
                } else if (ReefscapeController.lowAlgae()) {
                    state = LowAlgae
                }
            }
            LowAlgae -> {
                if (ReefscapeController.home()) {
                    state = Home
                } else if (ReefscapeController.highAlgae()) {
                    state = HighAlgae
                }
            }
            Net -> {
                if (ReefscapeController.home()) {
                    state = Home
                }
            }
            ReadyToClimb -> {
                if (ReefscapeController.climb()) {
                    state = FullyClimbed
                } else if (ReefscapeController.home()) {
                    state = Home
                }
            }
            L1,
            L2,
            L3,
            L4 -> processCoralScoring()
            PreScoreL4,
            FullyClimbed -> Unit
        }
    }

    private fun processCoralScoring() {
        if (ReefscapeController.home()) {
            state = Home
        } else if (ReefscapeController.l1()) {
            state = L1
        } else if (ReefscapeController.l2()) {
            state = L2
        } else if (ReefscapeController.l3()) {
            state = L3
        } else if (ReefscapeController.l4()) {
            state = L4
        }
    }

    fun periodic() {
        Elevator.periodic()
        Pivot.periodic()
        Wrist.periodic()
        Logger.recordOutput("superstructure/state", state.name)
    }

    fun simulationPeriodic() {
        Elevator.simulationPeriodic()
        Pivot.simulationPeriodic()
        Wrist.simulationPeriodic()
    }
}
