package frc.robot.subsystems

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.degrees
import frc.robot.lib.inches

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
        AlgaeGroundPickup(26.degrees, 0.25.inches, 181.35.degrees),
        Processor(10.7.degrees, 0.25.inches, 86.4.degrees),
        HighAlgaeIntake(100.2.degrees, 19.33.inches, 13.25.degrees),
        LowAlgaeIntake(104.degrees, 0.25.inches, 19.2125.degrees),
        AlgaeNet(91.degrees, 53.5.inches, 47.7675.degrees),
        ReadyToClimb(70.degrees, 0.25.inches, 180.degrees),
        FullyClimbed(5.degrees, 0.25.inches, 90.degrees),
    }
}
