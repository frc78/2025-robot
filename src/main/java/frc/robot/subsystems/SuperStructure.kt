package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.DeferredCommand
import frc.robot.lib.*
import frc.robot.lib.ScoreSelector.SelectedLevel
import frc.robot.subsystems.drivetrain.Chassis
import kotlin.math.atan
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

/** @property pivotAngle: Angle of the pivot from horizontal */
enum class RobotState(val pivotAngle: Angle, val elevatorHeight: Distance, val wristAngle: Angle) {
    Stow(40.degrees, 0.25.inches, 22.5.degrees),
    AlgaeStorage(60.degrees, 0.25.inches, 22.5.degrees),
    L1(40.degrees, 0.25.inches, 186.75.degrees),
    L2(93.5.degrees, 0.25.inches, 38.degrees),
    L3(92.6.degrees, 17.25.inches, 38.degrees),
    L4(92.5.degrees, 46.inches, 29.25.degrees),
    CoralStation(62.92.degrees, 0.25.inches, 176.degrees),
    NewCoralStation(67.1.degrees, 0.1.inches, 185.8.degrees), // pivot 69.1
    AlgaeGroundPickup(30.degrees, 0.25.inches, 181.35.degrees),
    Processor(23.degrees, 0.25.inches, 113.625.degrees),
    HighAlgaeIntake(100.2.degrees, 19.33.inches, 13.25.degrees),
    LowAlgaeIntake(104.degrees, 0.25.inches, 19.2125.degrees),
    AlgaeNet(91.degrees, 53.5.inches, 47.7675.degrees),
    ReadyToClimb(70.degrees, 0.25.inches, 180.degrees),
    FullyClimbed(5.degrees, 0.25.inches, 90.degrees),
    CoralStorage(62.92.degrees, 0.25.inches, Wrist.lowerLimit),
}

object SuperStructure {

    val atPosition
        get() = Pivot.atPosition && Elevator.atPosition && Wrist.atPosition

    // Command factory to go to a specific robot state
    fun goToMoveElevatorAndPivotTogether(state: RobotState): Command =
        Pivot.goTo(state)
            .andThen(Elevator.goTo(state))
            .andThen(Wrist.goTo(state))
            .withName("Go to $state all at once")

    enum class SmartGoToStates {
        ELEVATOR_GOING_UP,
        ELEVATOR_GOING_DOWN,
        OTHER,
    }

    // Command factory to go to a specific robot state
    fun smartGoTo(state: RobotState): Command =
        Commands.select(
            mapOf(
                SmartGoToStates.ELEVATOR_GOING_UP to goToMovePivotFirst(state),
                SmartGoToStates.ELEVATOR_GOING_DOWN to goToMoveElevatorFirst(state),
                SmartGoToStates.OTHER to goToMoveElevatorAndPivotTogether(state),
            )
        ) {
            when {
                Elevator.isStowed && state.elevatorHeight > Elevator.IS_STOWED_THRESHOLD ->
                    SmartGoToStates.ELEVATOR_GOING_UP
                Elevator.isStowed && state.elevatorHeight < Elevator.IS_STOWED_THRESHOLD ->
                    SmartGoToStates.ELEVATOR_GOING_DOWN
                else -> SmartGoToStates.OTHER
            }
        }

    // Safely retract from getting algae from reef by moving pivot down a bit first
    fun retractWithAlgae(): Command =
        Pivot.goTo(RobotState.AlgaeStorage)
            .andWait { Pivot.angle < 94.degrees }
            .andThen(
                Elevator.goTo(RobotState.AlgaeStorage)
                    .alongWith(Wrist.goTo(RobotState.AlgaeStorage))
            )

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
                    .andWait { Pivot.angle > 80.degrees }
                    .andThen(Elevator.goTo(state).andWait { Elevator.position > 30.inches })
                    .andThen(Wrist.goTo(state))
            else -> smartGoTo(state)
        }

    val goToSelectedLevel by command {
        Commands.select(
            mapOf(
                Level.L1 to goToScoreCoral(RobotState.L1),
                Level.L2 to goToScoreCoral(RobotState.L2),
                Level.L3 to goToScoreCoral(RobotState.L3),
                Level.L4 to goToScoreCoral(RobotState.L4),
            )
        ) {
            SelectedLevel
        }
    }

    val goToScoreCoralWhenClose by command {
        Commands.sequence(Commands.waitUntil { Chassis.isWithinGoal(1.5) }, goToSelectedLevel)
    }

    private fun goToMoveElevatorFirst(state: RobotState): Command =
        Wrist.goTo(state)
            .alongWith(
                Elevator.goTo(state).andWait { Elevator.position < Elevator.MOVE_PIVOT_THRESHOLD }
            )
            .andThen(Pivot.goTo(state))
            .withName("Go to $state elevator first")

    private fun goToMovePivotFirst(state: RobotState): Command =
        Wrist.goTo(state)
            .alongWith(Pivot.goTo(state).andWait { Pivot.canExtendElevator })
            .andThen(Elevator.goTo(state))
            .withName("Go to $state pivot first")

    val scoreCoralOnSelectedBranch by command {
        Commands.sequence(
            goToSelectedLevel,
            Commands.waitUntil { atPosition },
            Intake.scoreCoral,
            smartGoTo(RobotState.CoralStation),
        )
    }

    private val goToCalculatedAlgaeHeight by command {
        Commands.select(
            mapOf(
                true to smartGoTo(RobotState.HighAlgaeIntake),
                false to smartGoTo(RobotState.LowAlgaeIntake),
            )
        ) {
            FieldPoses.closestAlgaeIsHigh
        }
    }

    val goToNetWhileAligning by command {
        Commands.waitUntil { Chassis.isWithinGoal(1.75) }.andThen(smartGoTo(RobotState.AlgaeNet))
    }

    val retrieveAlgaeFromReef by command {
        goToCalculatedAlgaeHeight
            .withDeadline(Intake.intakeAlgaeThenHold())
            .andThen(retractWithAlgae())
    }

    val retrieveHighAlgaeFromReef by command {
        smartGoTo(RobotState.HighAlgaeIntake)
            .withDeadline(Intake.intakeAlgaeThenHold())
            .andThen(retractWithAlgae())
    }

    val autoScoreAlgaeInNet by command {
        smartGoTo(RobotState.AlgaeNet)
            .andWait { atPosition }
            .andThen(Intake.scoreAlgae)
            .andThen(smartGoTo(RobotState.NewCoralStation))
            .onlyIf { Intake.detectAlgaeByCurrent() }
    }

    // TODO confirm accuracy on real field, tune as needed
    // todo Introduce limitation so it won't extend past 18"
    private val ELEVATOR_LENGTH: Distance = 0.71.meters // 28.5.inches //0003407122.inches
    private val PIVOT_TO_CENTER: Distance = 9.5.inches
    private val CLAW_HORIZONTAL: Distance = 0.38.meters // 15.551181.inches
    private val PIVOT_BIAS: Angle = 0.0.degrees // 4.8.degrees // 0.8!! if needed
    private val INTAKE_HEIGHT: Distance =
        (ELEVATOR_LENGTH + RobotState.NewCoralStation.elevatorHeight) *
            sin(RobotState.NewCoralStation.pivotAngle.radians - PIVOT_BIAS.radians)

    val reachToIntake by command {
        DeferredCommand(
            {
                // Horizontal distance from Pivot joint to desired location of Wrist joint
                val baseDist: Distance =
                    FieldGeometry.distanceToClosestLine(
                            FieldGeometry.CORAL_STATIONS,
                            Chassis.state.Pose.translation,
                        )
                        .meters + PIVOT_TO_CENTER - CLAW_HORIZONTAL

                // Trig calculation on target height and distance away, then account for Pivot bias
                // (zeroed below horizontal)
                val targetPivotAngle: Angle =
                    atan(INTAKE_HEIGHT.inches / baseDist.inches).radians + PIVOT_BIAS

                // Pythagorean theorem, then subtract retracted height of elevator to give the
                // necessary extension
                val targetElevatorHeight: Distance =
                    sqrt(baseDist.inches.pow(2) + INTAKE_HEIGHT.inches.pow(2)).inches -
                        ELEVATOR_LENGTH

                // Needs to decrease by the amount the Pivot angle decreases relative to the base
                // preset
                val targetWristAngle: Angle =
                    RobotState.CoralStation.wristAngle -
                        (RobotState.CoralStation.pivotAngle - targetPivotAngle)

                // Distance from frame to coral station; should not extend if > 18in
                val frameToStation: Distance =
                    FieldGeometry.distanceToClosestLine(
                            FieldGeometry.CORAL_STATIONS,
                            Chassis.state.Pose.translation,
                        )
                        .meters - 14.inches

                //            SmartDashboard.putNumber("Raw Station Dist", )
                //            SmartDashboard.putNumber("height value in", INTAKE_HEIGHT.inches)
                //            SmartDashboard.putNumber("base dist m", baseDist.meters)
                //            SmartDashboard.putNumber("tgt pivot deg", targetPivotAngle.degrees)
                //            SmartDashboard.putNumber("tgt elevator in",
                // targetElevatorHeight.inches)
                //            SmartDashboard.putNumber("tgt wrist deg", targetWristAngle.degrees)

                // Waiting for Pivot not necessary at shorter (legal) distances
                //            Pivot.goToRaw(targetPivotAngle)
                //                .andWait { Pivot.atPosition }
                //                .andThen(
                //                    Elevator.goToRaw(targetElevatorHeight)
                //                        .alongWith(Wrist.goToRaw(targetWristAngle)))

                // Extend if it doesn't violate extension rule
                ConditionalCommand(
                    Commands.parallel(
                        Pivot.goToRaw(targetPivotAngle),
                        Elevator.goToRaw(targetElevatorHeight),
                        Wrist.goToRaw(targetWristAngle),
                    ),
                    Commands.idle(),
                ) {
                    frameToStation <= 18.inches
                }
            },
            setOf(Pivot, Wrist, Elevator),
        )
    }
}
