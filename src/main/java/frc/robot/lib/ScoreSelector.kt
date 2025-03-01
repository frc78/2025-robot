package frc.robot.lib

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Commands
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
    var selectedLevel: Level = Level.L2
        set(value) {
            field = value
            Logger.recordOutput("selectedLevel", value.name)
        }

    var selectedBranch: Branch = Branch.LEFT
        set(value) {
            field = value
            Logger.recordOutput("selectedBranchSide", value.name)
        }

    val levelUp by command { Commands.runOnce({ selectedLevel = selectedLevel.next() }) }

    val levelDown by command { Commands.runOnce({ selectedLevel = selectedLevel.previous() }) }

    val selectLeftBranch by command { Commands.runOnce({ selectedBranch = Branch.LEFT }) }
    val selectRightBranch by command { Commands.runOnce({ selectedBranch = Branch.RIGHT }) }

    private val table = NetworkTableInstance.getDefault().getTable("score_selector")
    private val l1Selected = table.getBooleanTopic("l1Selected").publish()
    private val l2lSelected = table.getBooleanTopic("l2lSelected").publish()
    private val l2rSelected = table.getBooleanTopic("l2rSelected").publish()
    private val l3lSelected = table.getBooleanTopic("l3lSelected").publish()
    private val l3rSelected = table.getBooleanTopic("l3rSelected").publish()
    private val l4lSelected = table.getBooleanTopic("l4lSelected").publish()
    private val l4rSelected = table.getBooleanTopic("l4rSelected").publish()

    fun telemeterize() {
        l1Selected.set(selectedLevel == Level.L1)
        l2lSelected.set(selectedLevel == Level.L2 && selectedBranch == Branch.LEFT)
        l2rSelected.set(selectedLevel == Level.L2 && selectedBranch == Branch.RIGHT)
        l3lSelected.set(selectedLevel == Level.L3 && selectedBranch == Branch.LEFT)
        l3rSelected.set(selectedLevel == Level.L3 && selectedBranch == Branch.RIGHT)
        l4lSelected.set(selectedLevel == Level.L4 && selectedBranch == Branch.LEFT)
        l4rSelected.set(selectedLevel == Level.L4 && selectedBranch == Branch.RIGHT)
    }
}
