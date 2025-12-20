package frc.robot.auto

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.FieldPoses
import frc.robot.lib.FieldPoses.Branch.*
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Intake.IntakeState.*
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.SuperStructure.SuperStructureState.*
import frc.robot.subsystems.drivetrain.Chassis

/* Auto that works with 2056's path
 *
 * The auto loops steps 1-5 4 times, scoring on a different branch each loop.
 * The order of the scoring is controlled by the `branches` list.
 * This auto works for both left and right auto sides. */
object OPAuto {
    val timer = Timer()
    var step = 0

    val branches = listOf(listOf(F, I), listOf(C, L), listOf(D, K), listOf(E, J))
    var branchIndex = 0

    var currentBranchPose = E.pose

    fun init() {
        branchIndex = 0
        step = 0
    }

    fun runAuto() {
        when (step) {
            0 -> {
                // Prepare to score L4
                Intake.state = IntakeCoral
                SuperStructure.state = PreScoreL4
                currentBranchPose =
                    Chassis.state.Pose.nearest(branches[branchIndex].map { it.pose })
                // Drive forward
                step++
            }
            1 -> {
                // Drive toward branch and go to L4 when close
                if (Chassis.driveToPose(currentBranchPose, 1.0)) {
                    SuperStructure.state = L4
                    step++
                }
            }
            2 -> {
                // Start scoring sequence 5 cm away
                if (Chassis.driveToPose(currentBranchPose, .05)) {
                    timer.restart()
                    step++
                }
            }
            3 -> {
                // Score coral for .2 seconds
                Chassis.driveToPose(currentBranchPose, 0.0)
                Intake.state = EjectCoral
                if (timer.hasElapsed(0.2)) {
                    Intake.state = IntakeCoral
                    step++
                    if (branchIndex == 3) {
                        // Finished all branches
                        step = 6
                    }
                }
            }
            4 -> {
                SuperStructure.state = CoralStation
                Chassis.driveToPose(FieldPoses.closestCoralStation, 0.0)
                if (Intake.holdingCoral) {
                    // Get the next branch
                    branchIndex++
                    currentBranchPose =
                        Chassis.state.Pose.nearest(branches[branchIndex].map { it.pose })
                    step++
                }
            }
            5 -> {
                // Prepare to score L4 when away from coral station
                if (Chassis.driveToPose(currentBranchPose, 3.0)) {
                    SuperStructure.state = PreScoreL4
                    step = 1
                }
            }
            6 -> {
                Chassis.setControl(SwerveRequest.Idle())
                Intake.state = Idle
                SuperStructure.state = Home
            }
        }
    }
}
