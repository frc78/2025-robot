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

    var state = LedState.Disabled

    init {
        led.setColorOrder(AddressableLED.ColorOrder.kRGB)
        LEDPattern.solid(Color.kWhite).applyTo(buffer)
        led.setLength(buffer.length)
        led.setData(buffer)
        led.start()
    }

    fun stateMachine() {
        when (state) {
            LedState.Disabled -> {
                setColor(Color.kWhite)
            }
            LedState.Idle -> {
                setColor(Color.kBlack)
                if (
                    Intake.state == Intake.IntakeState.HoldCoral ||
                        Intake.state == Intake.IntakeState.HoldAlgae
                ) {
                    state = LedState.GamePieceAcquired
                }
            }
            LedState.GamePieceAcquired -> {
                setColor(Color.kGreen)
                if (
                    Intake.state != Intake.IntakeState.HoldCoral &&
                        Intake.state != Intake.IntakeState.HoldAlgae
                ) {
                    state = LedState.Idle
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
