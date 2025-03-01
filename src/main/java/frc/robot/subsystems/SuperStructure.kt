package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.DeferredCommand
import frc.robot.Robot
import frc.robot.lib.ScoreSelector.SelectedLevel
import frc.robot.lib.command
import frc.robot.lib.degrees
import frc.robot.lib.inches

/** @property pivotAngle: Angle of the pivot from horizontal */
enum class RobotState(val pivotAngle: Angle, val elevatorHeight: Distance, val wristAngle: Angle) {
    Stow(0.degrees, 0.25.inches, 8.degrees),
    PreScore(70.degrees, 0.25.inches, 125.degrees),
    L1(60.degrees, 0.25.inches, 120.degrees),
    L2(69.degrees, 0.25.inches, 22.67.degrees),
    L3(78.degrees, 20.inches, 20.degrees),
    L4(82.degrees, 50.13.inches, 13.62.degrees),
    Net(82.degrees, 46.inches, 100.degrees),
    CoralStation(65.92.degrees, 0.25.inches, 165.9.degrees),
    AlgaeGroundPickup(18.degrees, 3.inches, 30.degrees),
    CoralGroundPickup(5.degrees, 5.inches, 74.degrees),
    Processor(0.degrees, 0.inches, 0.degrees),
    HighAlgaeIntake(84.degrees, 17.33.inches, 10.degrees),
    LowAlgaeIntake(84.degrees, 0.25.inches, 10.degrees),
    AlgaeNet(82.97.degrees, 51.61.inches, 39.46.degrees),
    ReadyToClimb(78.degrees, 0.25.inches, 120.degrees), // todo test pivot angle
    FullyClimbed(25.degrees, 0.25.inches, 120.degrees), // todo test pivot angle
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
            .withName("Go to $state all at once")

    // Command for the superstructure to automatically retract to CoralStation preset after outtaking a gamepiece
    fun retractAfterScoring(): Command =
        DeferredCommand(
            {
                if (Elevator.position > RobotState.CoralStation.elevatorHeight) {
                    // If elevator is moving down, wait for wrist to get out of the way
                    Wrist.goToAndWaitUntilAtAngle(RobotState.CoralStation) // todo could prolly get away with less wrist angle to retract quicker
                        .andThen(goToMoveElevatorFirst(RobotState.CoralStation))
                } else {
                    // Otherwise, use normal logic
                    smartGoTo(RobotState.CoralStation)
                }
            },
            setOf(Pivot, Elevator, Wrist)
        ).withName("Superstructure retracting after scoring")

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
        Pivot.goToAndWaitUntilVertical(state)
            .andThen(Elevator.goToAndWaitUntilAtHeight(state))
            .andThen(Wrist.goTo(state))
            .withName("Go to $state pivot first")

    fun goToScoreReefFromPreScore(state: RobotState): Command =
        DeferredCommand(
            {
                if (state == RobotState.L4 || state == RobotState.L1) {
                    // If going L4, can do wrist last bc there's nothing above it to hit
                    // If going L1, scoring out the front with no elevator so no worries
                    smartGoTo(state)
                } else {
                    // If going L2 or L3, set elevator and wrist first, then move pivot the last few degrees
                    Elevator.goToAndWaitUntilAtHeight(state)
                        .alongWith(Wrist.goToAndWaitUntilAtAngle(state))
                        .andThen(Pivot.goTo(state))
                        .withName("Go to score reef at $state from PreScore")
                }
            },
            setOf(Pivot, Elevator, Wrist)
        )
}
