package frc.robot.lib

import edu.wpi.first.networktables.NetworkTableInstance
import frc.robot.subsystems.RobotState
import org.littletonrobotics.junction.Logger

enum class Branch {
    LEFT,
    RIGHT,
}

/* There probably is a better way to structure this, but the reason why I
 * made this a separate enum is because this is meant as a selection value/logic state
 * while the one above is meant to represent robot state*/
enum class Level(val state: RobotState) {
    L1(RobotState.L1),
    L2(RobotState.L2),
    L3(RobotState.L3),
    L4(RobotState.L4);

    fun next() = Level.entries.getOrNull(ordinal + 1) ?: L4

    fun previous() = Level.entries.getOrNull(ordinal - 1) ?: L1
}

object ScoreSelector {
    var SelectedLevel: Level = Level.L4
        set(value) {
            field = value
            Logger.recordOutput("selectedLevel", value.name)
        }

    var SelectedBranch: Branch = Branch.LEFT
        set(value) {
            field = value
            Logger.recordOutput("selectedBranchSide", value.name)
        }

    fun levelUp() {
        SelectedLevel = SelectedLevel.next()
    }

    fun levelDown() {
        SelectedLevel = SelectedLevel.previous()
    }

    private val table = NetworkTableInstance.getDefault().getTable("score_selector")
    private val l1Selected = table.getBooleanTopic("l1Selected").publish()
    private val l2lSelected = table.getBooleanTopic("l2lSelected").publish()
    private val l2rSelected = table.getBooleanTopic("l2rSelected").publish()
    private val l3lSelected = table.getBooleanTopic("l3lSelected").publish()
    private val l3rSelected = table.getBooleanTopic("l3rSelected").publish()
    private val l4lSelected = table.getBooleanTopic("l4lSelected").publish()
    private val l4rSelected = table.getBooleanTopic("l4rSelected").publish()

    fun telemeterize() {
        l1Selected.set(SelectedLevel == Level.L1)
        l2lSelected.set(SelectedLevel == Level.L2 && SelectedBranch == Branch.LEFT)
        l2rSelected.set(SelectedLevel == Level.L2 && SelectedBranch == Branch.RIGHT)
        l3lSelected.set(SelectedLevel == Level.L3 && SelectedBranch == Branch.LEFT)
        l3rSelected.set(SelectedLevel == Level.L3 && SelectedBranch == Branch.RIGHT)
        l4lSelected.set(SelectedLevel == Level.L4 && SelectedBranch == Branch.LEFT)
        l4rSelected.set(SelectedLevel == Level.L4 && SelectedBranch == Branch.RIGHT)
    }
}
