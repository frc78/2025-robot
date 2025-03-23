package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.DeferredCommand
import frc.robot.lib.FieldPoses
import frc.robot.lib.ScoreSelector.SelectedLevel
import frc.robot.lib.andWait
import frc.robot.lib.command
import frc.robot.lib.degrees
import frc.robot.lib.inches

/** @property pivotAngle: Angle of the pivot from horizontal */
enum class RobotState(val pivotAngle: Angle, val elevatorHeight: Distance, val wristAngle: Angle) {
    Stow(40.degrees, 0.25.inches, 22.5.degrees), // not currently used
    AlgaeStorage(60.degrees, 0.25.inches, 22.5.degrees),
    L1(40.degrees, 0.25.inches, 186.75.degrees),
    L2(93.5.degrees, 0.25.inches, 34.5.degrees),
    L3(92.6.degrees, 17.25.inches, 33.5.degrees),
    L4(92.5.degrees, 46.inches, 29.25.degrees),
    CoralStation(62.92.degrees, 0.25.inches, 176.degrees),
    AlgaeGroundPickup(30.degrees, 0.25.inches, 181.35.degrees),
    Processor(23.degrees, 0.25.inches, 113.625.degrees),
    HighAlgaeIntake(97.2.degrees, 19.33.inches, 11.25.degrees),
    LowAlgaeIntake(100.degrees, 0.25.inches, 17.2125.degrees),
    AlgaeNet(91.degrees, 53.5.inches, 47.7675.degrees),
    ReadyToClimb(70.degrees, 0.25.inches, 180.degrees),
    FullyClimbed(5.degrees, 0.25.inches, 90.degrees),
    CoralStorage(
        62.92.degrees,
        0.25.inches,
        Wrist.lowerLimit,
    ), // same as coral station but with wrist over
}

object SuperStructure {

    val atPosition
        get() = Pivot.atPosition && Elevator.atPosition && Wrist.atPosition

    val goToSelectedLevel by command {
        DeferredCommand({ goToScoreCoral(SelectedLevel.state) }, setOf(Elevator, Pivot, Wrist))
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
                setOf(Elevator, Pivot, Wrist),
            )
            .withName("Smart Go To ${state.name}")

    // Safely retract from getting algae from reef by moving pivot down a bit first
    fun retractWithAlgae(): Command =
        Pivot.goTo(RobotState.AlgaeStorage)
            .andWait { Pivot.angle < 90.degrees }
            .andThen(Elevator.goTo(RobotState.AlgaeStorage))
            .alongWith(Wrist.goTo(RobotState.AlgaeStorage))

    // Do fancier experimental movement to avoid hitting coral on branches for L2, L3, L4
    fun goToScoreCoral(state: RobotState): Command =
        when (state) {
            RobotState.L2 ->
                // Move wrist and elevator first, wait for wrist before moving pivot
                Elevator.goTo(state)
                    .andThen(Wrist.goTo(state).andWait { Wrist.angle < 140.degrees })
                    .andThen(Pivot.goTo(state))

            RobotState.L3 ->
                Commands.parallel(Wrist.goTo(state), Elevator.goTo(state), Pivot.goTo(state))

            RobotState.L4 ->
                Pivot.goTo(state)
                    .andWait { Pivot.canExtendElevator }
                    .andThen(Elevator.goTo(state).andWait { Elevator.position > 20.inches })
                    .andThen(Wrist.goTo(state))
            else -> smartGoTo(state)
        }

    fun goToMoveElevatorFirst(state: RobotState): Command =
        Wrist.goTo(state)
            .alongWith(
                Elevator.goTo(state).andWait { Elevator.position < Elevator.MOVE_PIVOT_THRESHOLD }
            )
            .andThen(Pivot.goTo(state))
            .withName("Go to $state elevator first")

    fun goToMovePivotFirst(state: RobotState): Command =
        Wrist.goTo(state)
            .alongWith(Pivot.goTo(state).andWait { Pivot.canExtendElevator })
            .andThen(Elevator.goTo(state))
            .withName("Go to $state pivot first")

    val scoreCoralOnSelectedBranch by command {
//        Commands.defer(
//            {
//                goToScoreCoral(SelectedLevel.state)
//                    .andWait { atPosition }
//                    .andThen(Intake.scoreCoral)
//                    .andThen(smartGoTo(RobotState.CoralStation))
//            },
//            setOf(Pivot, Elevator, Wrist, Intake),
//        )
        Commands.sequence(
            Commands.defer({goToScoreCoral(SelectedLevel.state)}, setOf(Pivot, Elevator, Wrist)),
            Commands.waitUntil { atPosition },
            Intake.scoreCoral,
            smartGoTo(RobotState.CoralStation)
        )
    }

    val goToCalculatedAlgaeHeight by command {
        Commands.defer(
            {
                smartGoTo(
                    if (FieldPoses.closestAlgaeIsHigh) RobotState.HighAlgaeIntake
                    else RobotState.LowAlgaeIntake
                )
            },
            setOf(Pivot, Elevator, Wrist),
        )
    }

    val retrieveAlgaeFromReef by command {
        goToCalculatedAlgaeHeight.withDeadline(Intake.intakeAlgaeThenHold())
            .andThen(retractWithAlgae())
    }

    val autoScoreAlgaeInNet by command {
        smartGoTo(RobotState.AlgaeNet)
            .andWait { atPosition }
            .andThen(Intake.scoreAlgae)
            .andThen(smartGoTo(RobotState.ReadyToClimb))
            .onlyIf { Intake.detectAlgaeByCurrent() }
    }
}
