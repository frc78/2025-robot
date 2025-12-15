package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color

object LEDSubsystem {

    enum class LedState {
        Disabled,
        Idle,
        GamePieceAcquired,
        Aligned,
    }

    private val led = AddressableLED(0)
    private val buffer = AddressableLEDBuffer(30)

    var currentState = LedState.Disabled

    init {
        led.setColorOrder(AddressableLED.ColorOrder.kRGB)
        LEDPattern.solid(Color.kWhite).applyTo(buffer)
        led.setLength(buffer.length)
        led.setData(buffer)
        led.start()
    }

    fun stateMachine() {
        when (currentState) {
            LedState.Disabled -> {
                setColor(Color.kWhite)
            }
            LedState.Idle -> {
                setColor(Color.kBlack)
                if (
                    Intake.currentState == Intake.IntakeState.HoldCoral ||
                        Intake.currentState == Intake.IntakeState.HoldAlgae
                ) {
                    currentState = LedState.GamePieceAcquired
                }
            }
            LedState.GamePieceAcquired -> {
                setColor(Color.kGreen)
                if (
                    Intake.currentState != Intake.IntakeState.HoldCoral &&
                        Intake.currentState != Intake.IntakeState.HoldAlgae
                ) {
                    currentState = LedState.Idle
                }
            }
            LedState.Aligned -> {
                setColor(Color.kBlue)
            }
        }
    }

    private fun setColor(color: Color) {
        LEDPattern.solid(color).applyTo(buffer)
        led.setData(buffer)
    }
}
