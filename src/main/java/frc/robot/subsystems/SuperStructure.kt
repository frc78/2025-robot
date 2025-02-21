package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.DeferredCommand
import frc.robot.lib.degrees
import frc.robot.lib.inches

/** @property pivotAngle: Angle of the pivot from horizontal */
enum class RobotState(val pivotAngle: Angle, val elevatorHeight: Distance, val wristAngle: Angle) {
    L1(60.degrees, 0.inches, 120.degrees),
    L2(75.degrees, 6.inches, 110.degrees),
    L3(78.degrees, 20.inches, 100.degrees),
    L4(82.degrees, 46.inches, 100.degrees),
    Net(82.degrees, 46.inches, 100.degrees),
    CoralStation(54.degrees, 0.inches, 19.degrees),
    AlgaeGroundPickup(18.degrees, 3.inches, 30.degrees),
    CoralGroundPickup(5.degrees, 5.inches, 74.degrees),
    Processor(0.degrees, 0.inches, 0.degrees),
    HighAlgaeIntake(0.degrees, 0.inches, 0.degrees),
    LowAlgaeIntake(0.degrees, 0.inches, 0.degrees),
    ReadyToClimb(0.degrees, 0.inches, 0.degrees),
    FullyClimbed(0.degrees, 0.inches, 0.degrees),
    AlgaeStorage(0.degrees, 0.inches, 0.degrees),
    CoralStorage(0.degrees, 0.inches, 0.degrees),
}

object SuperStructure {
    init {
        RobotState.entries.forEach { SmartDashboard.putData(smartGoTo(it)) }
    }

    // Command factory to go to a specific robot state
    fun goTo(state: RobotState): Command =
        Pivot.goTo(state)
            .andThen(Elevator.goTo(state))
            .andThen(Wrist.goTo(state))
            .withName("Go to $state")

    // Old smartGoTo that doesn't account for moving between states where the elevator stays raised (or stowed)
//    fun smartGoTo(state: RobotState): Command =
//        ConditionalCommand(goToElevatorIsStowed(state), goToElevatorIsRaised(state), Elevator.isStowed)
//            .withName("Go to $state")

    // Command to go to a RobotState and have the elevator and pivot play nice
    fun smartGoTo(state: RobotState): Command =
        DeferredCommand({
            if (Elevator.isStowed.asBoolean && state.elevatorHeight > Elevator.IS_STOWED_THRESHOLD) {
                // if elevator is stowed and getting raised, move pivot before raising it
                goToElevatorIsStowed(state)
            } else if (!Elevator.isStowed.asBoolean && state.elevatorHeight < Elevator.IS_STOWED_THRESHOLD) {
                // if elevator is raised and getting stowed, lower it before moving pivot
                goToElevatorIsRaised(state)
            } else {
                // if elevator is not going from stowed to raised or vice versa, move everything at once
                goTo(state)
            }
        }, setOf(Pivot, Elevator, Wrist))

    fun goToElevatorIsRaised(state: RobotState): Command =
        Wrist.goTo(state)
            .andThen(Elevator.goToAndWaitUntilStowed(state))
            .andThen(Pivot.goTo(state))
            .withName("Go to $state")

    fun goToElevatorIsStowed(state: RobotState): Command =
        Wrist.goTo(state)
            .andThen(Pivot.goToAndWaitUntilVertical(state))
            .andThen(Elevator.goTo(state))
            .withName("Go to $state")
}
