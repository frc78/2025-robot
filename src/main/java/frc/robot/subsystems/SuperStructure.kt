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

/** @property pivotAngle: Angle of the pivot from horizontal */
enum class RobotState(val pivotAngle: Angle, val elevatorHeight: Distance, val wristAngle: Angle) {
    // pivot  elevator  wrist
    InFramePerimeter(45.degrees, 0.0.inches, 0.degrees),
    Stow(5.degrees, 0.25.inches, 20.degrees),
    PreScore(60.degrees, 0.25.inches, 20.degrees),
    L1(45.degrees, 0.25.inches, 166.degrees),
    L2(88.degrees, 0.25.inches, 32.1.degrees),
    L3(90.6.degrees, 19.4.inches, 29.8.degrees),
    L4(91.degrees, 48.inches, 26.degrees),
    IntermediaryL4(89.75.degrees, 48.inches, 120.degrees),
    Net(82.degrees, 46.inches, 100.degrees),
    CoralStation(65.92.degrees, 0.25.inches, 165.9.degrees),
    AlgaeGroundPickup(30.degrees, 0.25.inches, 161.2.degrees),
    CoralGroundPickup(13.89.degrees, 0.25.inches, 183.degrees),
    Processor(23.degrees, 0.25.inches, 101.degrees),
    HighAlgaeIntake(97.2.degrees, 17.33.inches, 10.degrees),
    LowAlgaeIntake(100.degrees, 0.25.inches, 15.3.degrees),
    AlgaeNet(91.degrees, 53.5.inches, 42.46.degrees),
    ReadyToClimb(78.degrees, 0.25.inches, 160.degrees),
    FullyClimbed(5.degrees, 0.25.inches, 80.degrees),
    AlgaeStorage(0.degrees, 0.inches, 0.degrees),
    CoralStorage(0.degrees, 0.inches, 0.degrees),

    /*
    PRESETS STILL TO GET!! as of March 1 2025
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

    val atPosition
        get() = Pivot.atPosition && Elevator.atPosition && Wrist.atPosition

    val goToSelectedLevel by command {
        DeferredCommand({ smartGoTo(SelectedLevel.state) }, setOf(Pivot, Elevator, Wrist))
    }

    // Command factory to go to a specific robot state
    fun goToMoveElevatorAndPivotTogether(state: RobotState): Command =
        Pivot.goTo(state)
            .andThen(Elevator.goTo(state))
            .andThen(Wrist.goTo(state))
            .withName("Go to $state all at once")

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

    // Do fancier experimental movement to avoid hitting coral on branches for L2, L3, L4
    fun goToScoreCoral(state: RobotState): Command =
        DeferredCommand(
            {
                // TODO check if in good state for this first? or safe to assume PreScore/Stow?
                when (state) {
                    RobotState.L2 ->
                        // Move wrist and elevator first, wait for wrist before moving pivot
                        Wrist.goToRawUntil(state.wristAngle) { Wrist.angle < 45.degrees }
                            .alongWith(Elevator.goTo(state))
                            .andThen(Pivot.goTo(state))
                    RobotState.L3 ->
                        // Move wrist and elevator first, wait for elevator before moving pivot
                        Wrist.goTo(state)
                            .alongWith(
                                Elevator.goToRawUntil(state.elevatorHeight) {
                                    Elevator.position > 10.inches
                                }
                            )
                            .andThen(Pivot.goTo(state))
                    RobotState.L4 ->
                        smartGoTo(
                                RobotState.IntermediaryL4
                            ) // same as L4 but has wrist pointing upwards
                            .andThen(Commands.waitUntil { atPosition })
                            .andThen(smartGoTo(state))
                    else -> smartGoTo(state)
                }
            },
            setOf(Pivot, Elevator, Wrist),
        )

    fun goToMoveElevatorFirst(state: RobotState): Command =
        Wrist.goTo(state)
            .alongWith(
                Elevator.goToRawUntil(state.elevatorHeight) {
                    Elevator.position < Elevator.MOVE_PIVOT_THRESHOLD
                }
            )
            .andThen(Pivot.goTo(state))
            .withName("Go to $state elevator first")

    fun goToMovePivotFirst(state: RobotState): Command =
        Wrist.goTo(state)
            .alongWith(Pivot.goToRawUntil(state.pivotAngle) { Pivot.canExtendElevator })
            .andThen(Elevator.goTo(state))
            .withName("Go to $state pivot first")
}
