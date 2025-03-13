package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
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
    Stow(40.degrees, 0.25.inches, 22.5.degrees), // not currently used
    AlgaeStorage(60.degrees, 0.25.inches, 22.5.degrees),
    L1(40.degrees, 0.25.inches, 186.75.degrees),
    L2(88.degrees, 0.25.inches, 30.9.degrees),
    L3(90.6.degrees, 16.4.inches, 33.5.degrees),
    L4(91.degrees, 45.8.inches, 29.25.degrees),
    IntermediaryL4(89.75.degrees, 48.inches, 120.degrees),
    Net(82.degrees, 46.inches, 100.degrees),
    CoralStation(62.92.degrees, 0.25.inches, 176.degrees),
    AlgaeGroundPickup(30.degrees, 0.25.inches, 181.35.degrees),
    CoralGroundPickup(13.89.degrees, 0.25.inches, 205.875.degrees),
    Processor(23.degrees, 0.25.inches, 113.625.degrees),
    HighAlgaeIntake(97.2.degrees, 19.33.inches, 11.25.degrees),
    LowAlgaeIntake(100.degrees, 0.25.inches, 17.2125.degrees),
    AlgaeNet(91.degrees, 53.5.inches, 47.7675.degrees),
    ReadyToClimb(70.degrees, 0.25.inches, 180.degrees),
    FullyClimbed(5.degrees, 0.25.inches, 90.degrees),
    CoralStorage(62.92.degrees, 0.25.inches, Wrist.lowerLimit), // same as coral station but with wrist over
}

object SuperStructure {

    init {
        //        RobotState.entries.forEach { SmartDashboard.putData(smartGoTo(it)) }
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

    // Safely retract from getting algae from reef by moving pivot down a bit first
    fun retractWithAlgae(): Command =
        Pivot.goToRawUntil(RobotState.AlgaeStorage.pivotAngle) { Pivot.angle < 90.degrees }
            .andThen(Elevator.goToRawUntil(RobotState.AlgaeStorage.elevatorHeight) { true })
            .alongWith(Wrist.goToRawUntil(RobotState.AlgaeStorage.wristAngle) { true })

    // Do fancier experimental movement to avoid hitting coral on branches for L2, L3, L4
    fun goToScoreCoral(state: RobotState): Command =
        DeferredCommand(
            {
                when (state) {
                    RobotState.L2 ->
                        // Move wrist and elevator first, wait for wrist before moving pivot
                        Elevator.goTo(state)
                            .andThen(
                                Wrist.goToRawUntil(state.wristAngle) { Wrist.angle < 140.degrees }
                            )
                            .andThen(Pivot.goTo(state))
                    RobotState.L3 ->
                        // Move wrist and elevator first, wait for elevator before moving pivot
                        Wrist.goTo(state)
                            .andThen(
                                Elevator.goToRawUntil(state.elevatorHeight) {
                                    Elevator.position > 0.inches
                                }
                            )
                            .andThen(Pivot.goTo(state))
                    RobotState.L4 ->
                        Pivot.goToRawUntil(state.pivotAngle) { Pivot.angle > 70.degrees }
                            .andThen(Elevator.goToRawUntil(state.elevatorHeight) { Elevator.position > 20.inches })
                            .andThen(Wrist.goTo(state))


                    // OLD and JERKY
                    //                        smartGoTo(state)
                    //                            .andThen(
                    //                                Wrist.goToRawUntil(120.degrees) {
                    // Elevator.position > 20.inches }
                    //                            ) //
                    //                            .andThen(Wrist.goToRawUntil(state.wristAngle) {
                    // true })
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
