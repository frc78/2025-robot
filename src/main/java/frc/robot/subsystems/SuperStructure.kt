package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import frc.robot.lib.degrees
import frc.robot.lib.inches

/** @property pivotAngle: Angle of the pivot from horizontal */
enum class RobotState(val pivotAngle: Angle, val elevatorHeight: Distance, val wristAngle: Angle) {
    L1(60.degrees, 0.inches, 120.degrees),
    L2(75.degrees, 6.inches, 110.degrees),
    L3(79.degrees, 16.inches, 144.degrees),
    L4(79.degrees, 46.inches, 144.degrees),
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
        RobotState.entries.forEach { SmartDashboard.putData(goTo(it)) }
    }

    // Command factory to go to a specific robot state
    fun goTo(state: RobotState): Command =
        Pivot.goTo(state)
            .andThen(Elevator.goTo(state))
            .andThen(Wrist.goTo(state))
            .withName("Go to $state")

    // Command factory to go to a specific robot state
    fun smartGoTo(state: RobotState): Command =
        ConditionalCommand(goToElevatorIsDown(state), goToElevatorIsUp(state), Elevator.isDown)
            .withName("Go to $state")

    //    fun smartGoTo(state: RobotState): Command = InstantCommand(
    //        {
    //            if (Elevator.isDown.asBoolean) {
    //                // if elevator is down, move pivot before raising it
    //                goToElevatorIsDown(state)
    //            } else if ((73.degrees < state.pivotAngle) && (state.pivotAngle < 90.degrees)) {
    //                // if elevator is up but the pivot is staying "vertical", can move everything
    // at once
    //                goTo(state)
    //            } else {
    //                // if elevator is up and the pivot is not staying vertical, lower elevator
    // before moving pivot
    //                goToElevatorIsUp(state)
    //            }
    //        }
    //    )

    fun goToElevatorIsUp(state: RobotState): Command =
        Wrist.goTo(state)
            .andThen(Elevator.goToAndWaitUntilDown(state))
            .andThen(Pivot.goTo(state))
            .withName("Go to $state")

    fun goToElevatorIsDown(state: RobotState): Command =
        Wrist.goTo(state)
            .andThen(Pivot.goToAndWaitUntilVertical(state))
            .andThen(Elevator.goTo(state))
            .withName("Go to $state")
}
