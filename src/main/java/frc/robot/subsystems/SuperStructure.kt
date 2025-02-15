package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Robot
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.Robot.driveController
import frc.robot.Robot.manipController

/** @property pivotAngle: Angle of the pivot from horizontal */
enum class RobotState(val pivotAngle: Angle, val elevatorHeight: Distance, val wristAngle: Angle) {
    L1(60.degrees, 0.inches, (-120).degrees),
    L2(75.degrees, 6.inches, (-110).degrees),
    L3(78.degrees, 20.inches, (-100).degrees),
    L4(82.degrees, 46.inches, (-100).degrees),
    Net(82.degrees, 46.inches, (-100).degrees),
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

/* There probably is a better way to structure this, but the reason why I
* made this a separate enum is because this is meant as a selection value/logic state
* while the one above is meant to represent robot state*/
enum class Level(val state: RobotState) {
    L1(RobotState.L1),
    L2(RobotState.L2),
    L3(RobotState.L3),
    L4(RobotState.L4),
}

fun Level.next(){
    return Level.entries.indexOf(this).let {
        if (it == Level.entries.lastIndex) Level.entries.first()
        else Level.entries[it + 1]
    }
}

fun Level.previous(){
    return Level.entries.indexOf(this).let {
        if (it == 0) Level.entries.last()
        else Level.entries[it - 1]
    }
}

enum class Branch {
    LEFT,
    RIGHT,
}

object SuperStructure {
    var selectedLevel: Level = Level.L2
    var selectedBranch: Branch = Branch.LEFT

    init {
        RobotState.entries.forEach { SmartDashboard.putData(goTo(it)) }

        configureSimpleLayout()
    }

    fun configureSimpleLayout() {
        manipController.povUp().onTrue(Commands.runOnce({ selectedLevel.next()}))
        manipController.povDown().onTrue(Commands.runOnce({ selectedLevel.previous()}))
        manipController.leftBumper().onTrue(Commands.runOnce({ selectedBranch = Branch.LEFT }))
        manipController.rightBumper().onTrue(Commands.runOnce({ selectedBranch = Branch.RIGHT }))
    }

    fun configureIntuitiveLayout() {

    }

    // Command factory to go to a specific robot state
    fun goTo(state: RobotState): Command =
        Pivot.goTo(state)
            .andThen(Elevator.goTo(state))
            .andThen(Wrist.goTo(state))
            .withName("Go to $state")
}
