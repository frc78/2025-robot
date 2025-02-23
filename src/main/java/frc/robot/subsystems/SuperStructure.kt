package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.DeferredCommand
import frc.robot.lib.ScoreSelector.SelectedLevel
import frc.robot.lib.command
import frc.robot.lib.degrees
import frc.robot.lib.inches

/** @property pivotAngle: Angle of the pivot from horizontal */
enum class RobotState(val pivotAngle: Angle, val elevatorHeight: Distance, val wristAngle: Angle) {
    // intake pivot 52.6 elevator 0.27 wrist 145.8
    // l2 pivot 75 elevator 6 wrist 11.78
    // l3 pivot 77.26 elevator 20.02 wrist 19.95
    // l4 pivot 81.65 elevator 46.02 wrist 20.65

    // intake 2 pivot 64.42 elevator 0 wrist 166.9
    // l4 2 pivot 81.73 elevator 50.13 wrist 13.62
    // l2 2 pivot 68.11 elevator 0.25 wrist 22.67
    Stow(0.degrees, 0.25.inches, 0.degrees),
    L1(60.degrees, 0.25.inches, 120.degrees),
    //    L2(75.degrees, 6.inches, 11.78.degrees),
    L2(69.degrees, 0.25.inches, 22.67.degrees),
    L3(78.degrees, 20.inches, 20.degrees),
    //    L4(82.degrees, 46.inches, 20.5.degrees),
    L4(82.degrees, 50.13.inches, 13.62.degrees),
    Net(82.degrees, 46.inches, 100.degrees),
    //    CoralStation(52.6.degrees, 0.25.inches, 145.8.degrees),
    CoralStation(65.92.degrees, 0.25.inches, 165.9.degrees),
    AlgaeGroundPickup(18.degrees, 3.inches, 30.degrees),
    CoralGroundPickup(5.degrees, 5.inches, 74.degrees),
    Processor(0.degrees, 0.inches, 0.degrees),
    HighAlgaeIntake(84.degrees, 17.33.inches, 8.5.degrees),
    LowAlgaeIntake(84.degrees, 0.25.inches, 8.5.degrees),
    AlgaeNet(82.97.degrees, 51.61.inches, 39.46.degrees),
    ReadyToClimb(0.degrees, 0.inches, 0.degrees),
    FullyClimbed(0.degrees, 0.inches, 0.degrees),
    AlgaeStorage(0.degrees, 0.inches, 0.degrees),
    CoralStorage(0.degrees, 0.inches, 0.degrees),
}

object SuperStructure {

    init {
        RobotState.entries.forEach { SmartDashboard.putData(smartGoTo(it)) }
    }

    val goToSelectedLevel by command {
        DeferredCommand({ smartGoTo(SelectedLevel.state) }, setOf(Pivot, Elevator, Wrist))
    }

    // Command factory to go to a specific robot state
    fun goToMoveElevatorAndPivotTogether(state: RobotState): Command =
        Pivot.goTo(state)
            .andThen(Elevator.goTo(state))
            .andThen(Wrist.goTo(state))
            .withName("Go to $state")

    // Command factory to go to a specific robot state
    fun smartGoTo(state: RobotState): Command =
        DeferredCommand(
                {
                    if (Elevator.isStowed && state.elevatorHeight > Elevator.IS_STOWED_THRESHOLD) {
                        // if elevator is stowed and getting raised, move pivot before raising it
                        goToMovePivotFirst(state)
                    } else if (
                        !Elevator.isStowed && state.elevatorHeight < Elevator.IS_STOWED_THRESHOLD
                    ) {
                        // if elevator is raised and getting stowed, lower it before moving pivot
                        println("Elevator going down, running goToElevatorIsRaised(state)")
                        goToMoveElevatorFirst(state)
                    } else {
                        // if elevator is not going from stowed to raised or vice versa, move
                        // everything
                        // at once
                        goToMoveElevatorAndPivotTogether(state)
                    }
                },
                setOf(Pivot, Elevator, Wrist),
            )
            .withName("Smart Go To ${state.name}")

    fun goToMoveElevatorFirst(state: RobotState): Command =
        Wrist.goTo(state)
            .andThen(Elevator.goToAndWaitUntilStowed(state))
            .andThen(Pivot.goTo(state))
            .withName("Go to $state")

    fun goToMovePivotFirst(state: RobotState): Command =
        Wrist.goTo(state)
            .andThen(Pivot.goToAndWaitUntilVertical(state))
            .andThen(Elevator.goTo(state))
            .withName("Go to $state")
}
