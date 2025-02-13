package frc.robot.subsystems

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.hardware.core.CoreCANcoder
import com.ctre.phoenix6.signals.GravityTypeValue
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.degrees

object Pivot : SubsystemBase("Pivot") {

    private val leader = TalonFX(9, "*")
    private val follower = TalonFX(10, "*")

    private val motionMagic = MotionMagicTorqueCurrentFOC(0.degrees)

    init {
        leader.configurator.apply {
            // Set soft limits to avoid breaking the pivot
            apply(
                SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(90.degrees)
                    .withReverseSoftLimitThreshold(0.degrees)
            )
            // Set feedback to encoder
            apply(
                FeedbackConfigs()
                    .withFusedCANcoder(CoreCANcoder(5, "*"))
                    .withRotorToSensorRatio(1920.0)
            )
            // Set feedforward and feedback gains
            apply(
                Slot0Configs()
                    .withKP(724.0)
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
            )
        }
        follower.setControl(Follower(9, true))
    }

    fun goTo(state: RobotState): Command =
        PrintCommand("Pivot going to $state - ${state.pivotAngle}")
            .alongWith(
                runOnce {
                    leader.setControl(motionMagic.withPosition(state.pivotAngle))
                    angle = state.pivotAngle
                }
            )

    var angle = 90.degrees
}
