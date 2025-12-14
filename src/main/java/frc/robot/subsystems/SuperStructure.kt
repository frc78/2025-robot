package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.bindings.ReefscapeController
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.subsystems.Intake.IntakeState
import frc.robot.subsystems.Intake.IntakeState.HoldCoral
import frc.robot.subsystems.SuperStructure.SuperStructureState.CoralStation
import frc.robot.subsystems.SuperStructure.SuperStructureState.FloorAlgae
import frc.robot.subsystems.SuperStructure.SuperStructureState.HighAlgae
import frc.robot.subsystems.SuperStructure.SuperStructureState.Home
import frc.robot.subsystems.SuperStructure.SuperStructureState.L1
import frc.robot.subsystems.SuperStructure.SuperStructureState.L2
import frc.robot.subsystems.SuperStructure.SuperStructureState.L3
import frc.robot.subsystems.SuperStructure.SuperStructureState.L4
import frc.robot.subsystems.SuperStructure.SuperStructureState.LowAlgae
import frc.robot.subsystems.SuperStructure.SuperStructureState.Net
import frc.robot.subsystems.SuperStructure.SuperStructureState.Processor
import java.util.function.BooleanSupplier

object SuperStructure : SubsystemBase("superstructure") {

    /** @property pivotAngle: Angle of the pivot from horizontal */
    enum class SuperStructureState(
        val pivotAngle: Angle,
        val elevatorHeight: Distance,
        val wristAngle: Angle,
    ) {
        Home(40.degrees, 0.25.inches, 22.5.degrees),
        L1(38.degrees, 0.25.inches, 186.75.degrees),
        L2(93.5.degrees, 0.25.inches, 39.degrees),
        L3(92.6.degrees, 17.25.inches, 38.degrees),
        L4(90.5.degrees, 47.inches, 29.25.degrees), // 92.5 pivot
        CoralStation(66.85.degrees, 0.1.inches, 185.8.degrees), // pivot 68.2 // 67.1 pivot 6/8
        FloorAlgae(26.degrees, 0.25.inches, 181.35.degrees),
        Processor(10.7.degrees, 0.25.inches, 86.4.degrees),
        HighAlgae(100.2.degrees, 19.33.inches, 13.25.degrees),
        LowAlgae(104.degrees, 0.25.inches, 19.2125.degrees),
        Net(91.degrees, 53.5.inches, 47.7675.degrees),
        ReadyToClimb(70.degrees, 0.25.inches, 180.degrees),
        FullyClimbed(5.degrees, 0.25.inches, 90.degrees);

        fun transition(to: SuperStructureState, condition: BooleanSupplier) {
            Trigger { currentState == this }.and(condition).onTrue(runOnce { currentState = to })
        }
    }

    private var previousState = Home
    var currentState = Home
        set(value) {
            previousState = field
            field = value
        }

    fun bind(state: SuperStructureState) = Trigger { currentState == state }

    val intakeIdle = { Intake.currentState == IntakeState.Idle }
    val intakeHasCoral = { Intake.currentState == HoldCoral }
    val intakeHasAlgae = { Intake.currentState == IntakeState.HoldAlgae }

    init {

        // Always allow going to home
        ReefscapeController.home().onTrue(runOnce { currentState = Home })

        Home.transition(L1, ReefscapeController.l1().and(intakeHasCoral))
        Home.transition(L2, ReefscapeController.l2().and(intakeHasCoral))
        Home.transition(L3, ReefscapeController.l3().and(intakeHasCoral))
        Home.transition(L4, ReefscapeController.l4().and(intakeHasCoral))
        Home.transition(CoralStation, ReefscapeController.coral().and(intakeIdle))
        Home.transition(FloorAlgae, ReefscapeController.floorAlgae().and(intakeIdle))
        Home.transition(Processor, ReefscapeController.process().and(intakeHasAlgae))
        Home.transition(HighAlgae, ReefscapeController.highAlgae().and(intakeIdle))
        Home.transition(LowAlgae, ReefscapeController.lowAlgae().and(intakeIdle))
        Home.transition(Net, ReefscapeController.net().and(intakeHasAlgae))

        // Coral scoring transitions
        // We don't need to check for having coral here because we already checked that to leave
        // home
        L1.transition(L2, ReefscapeController.l2())
        L1.transition(L3, ReefscapeController.l3())
        L1.transition(L4, ReefscapeController.l4())

        L2.transition(L1, ReefscapeController.l1())
        L2.transition(L3, ReefscapeController.l3())
        L2.transition(L4, ReefscapeController.l4())

        L3.transition(L1, ReefscapeController.l1())
        L3.transition(L2, ReefscapeController.l2())
        L3.transition(L4, ReefscapeController.l4())

        L4.transition(L1, ReefscapeController.l1())
        L4.transition(L2, ReefscapeController.l2())
        L4.transition(L3, ReefscapeController.l3())

        // For now, don't allow directly going between intake states and scoring states
        ReefscapeController.net()
            .and(intakeHasAlgae)
            .and { false }
            .onTrue(runOnce { currentState = Net })
        ReefscapeController.process()
            .and(intakeHasAlgae)
            .and { false }
            .onTrue(runOnce { currentState = Processor })
        ReefscapeController.l1()
            .and(intakeHasCoral)
            .and { false }
            .onTrue(runOnce { currentState = L1 })
        ReefscapeController.l2()
            .and(intakeHasCoral)
            .and { false }
            .onTrue(runOnce { currentState = L2 })
        ReefscapeController.l3()
            .and(intakeHasCoral)
            .and { false }
            .onTrue(runOnce { currentState = L3 })
        ReefscapeController.l4()
            .and(intakeHasCoral)
            .and { false }
            .onTrue(runOnce { currentState = L4 })
    }
}
