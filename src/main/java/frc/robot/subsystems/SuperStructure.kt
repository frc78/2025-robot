package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.lib.degrees
import frc.robot.lib.inches

enum class RobotState(val pivotAngle: Angle, val elevatorHeight: Distance, val wristAngle: Angle) {
    L1(60.degrees, 0.inches, (-120).degrees),
    L2(75.degrees, 6.inches, (-110).degrees),
    L3(78.degrees, 20.inches, (-100).degrees),
    L4(92.degrees, 46.inches, (-100).degrees),
    Net(92.degrees, 46.inches, (-100).degrees),
    CoralStation(60.degrees, 0.inches, 60.degrees),
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
}
