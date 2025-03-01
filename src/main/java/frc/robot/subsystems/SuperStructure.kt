package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.DeferredCommand
import frc.robot.lib.ScoreSelector.SelectedLevel
import frc.robot.lib.command
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.lib.seconds

/** @property pivotAngle: Angle of the pivot from horizontal */
enum class RobotState(val pivotAngle: Angle, val elevatorHeight: Distance, val wristAngle: Angle) {
    Stow(0.degrees, 0.25.inches, 8.degrees),
    PreScore(66.degrees, 0.25.inches, 166.degrees), // todo temporarily same as coral station
    L1(60.degrees, 0.25.inches, 120.degrees),
    L2(88.degrees, 0.25.inches, 30.586.degrees),
    L3(85.degrees, 20.inches, 22.5.degrees),
    L4(87.2.degrees, 48.inches, 20.degrees),
    Net(82.degrees, 46.inches, 100.degrees),
    CoralStation(65.92.degrees, 0.25.inches, 165.9.degrees),
    AlgaeGroundPickup(18.degrees, 3.inches, 30.degrees),
    CoralGroundPickup(5.degrees, 5.inches, 74.degrees),
    Processor(0.degrees, 0.inches, 0.degrees),
    HighAlgaeIntake(84.degrees, 17.33.inches, 10.degrees),
    LowAlgaeIntake(84.degrees, 0.25.inches, 10.degrees),
    AlgaeNet(82.97.degrees, 51.61.inches, 39.46.degrees),
    ReadyToClimb(78.degrees, 0.25.inches, 10.degrees), // todo test pivot angle
    FullyClimbed(5.degrees, 0.25.inches, 120.degrees), // todo test pivot angle
    AlgaeStorage(0.degrees, 0.inches, 0.degrees),
    CoralStorage(0.degrees, 0.inches, 0.degrees),

    /*
    PRESETS STILL TO GET!! as of March 1 2025
    - algae ground intake
    - algae processor
    - coral L1
    - pre-climb
    - climbed
    - pre-match frame perimeter
     */
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
            .withName("Go to $state all at once")

    // Command for the superstructure to automatically retract to CoralStation preset after outtaking a gamepiece
    fun retractAfterScoring(): Command =
        Wrist.goTo(RobotState.CoralStation)
            // Wait for elevator to be down *enough* to move pivot, not necessarily all the way with smooth motion
            .andThen(Elevator.goToRawUntil(RobotState.CoralStation.elevatorHeight) { Elevator.position < Elevator.MOVE_PIVOT_THRESHOLD })
            .andThen(Pivot.goTo(RobotState.CoralStation))
//        DeferredCommand(
//            {
//                if (Elevator.position > RobotState.CoralStation.elevatorHeight) {
//                    // If elevator is moving down, wait for wrist to get out of the way
//                    Wrist.goToRawUntil(102.degrees) { Wrist.angle > 100.degrees}
//                        // Wait for elevator to be down *enough* to move pivot, not necessarily all the way with smooth motion
//                        .andThen(Elevator.goToRawUntil(RobotState.CoralStation.elevatorHeight) { Elevator.position < Elevator.MOVE_PIVOT_THRESHOLD })
//                        .andThen(Pivot.goTo(RobotState.CoralStation))
//                        .andThen(Wrist.goTo(RobotState.CoralStation))
//                } else {
//                    // Otherwise, use normal logic
//                    smartGoTo(RobotState.CoralStation)
//                }
//            },
//            setOf(Pivot, Elevator, Wrist)
//        ).withName("Superstructure retracting after scoring")

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
                    // everything at once
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
            .withName("Go to $state elevator first")

    fun goToMovePivotFirst(state: RobotState): Command =
        Pivot.goToAndWaitUntilSetpoint(state)
            .andThen(Elevator.goToAndWaitUntilAtHeight(state))
            .andThen(Wrist.goTo(state))
            .withName("Go to $state pivot first")

    fun goToScoreReefFromPreScore(state: RobotState): Command =
        Wrist.goTo(state)
            .alongWith(Pivot.goToRawUntil(state.pivotAngle) { Pivot.canExtendElevator })
            .andThen(Elevator.goTo(state))
