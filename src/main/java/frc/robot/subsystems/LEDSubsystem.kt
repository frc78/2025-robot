package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.Level
import frc.robot.lib.ScoreSelector
import frc.robot.lib.command

object LEDSubsystem : SubsystemBase("led") {
    private val led = AddressableLED(0)
    private val buffer = AddressableLEDBuffer(30)

    init {
        led.setColorOrder(AddressableLED.ColorOrder.kRGB)
        LEDPattern.solid(Color.kWhite).applyTo(buffer)
        led.setLength(buffer.length)
        led.setData(buffer)
        led.start()
        defaultCommand = runOnce { setColorForSelectedLevel() }
    }

    fun setColorForSelectedLevel() {
        when (ScoreSelector.SelectedLevel) {
            Level.L1 -> setColor(Color.kGreen)
            Level.L2 -> setColor(Color.kRed)
            Level.L3 -> setColor(Color.kBlue)
            Level.L4 -> setColor(Color.kGreen)
        }
    }

    private fun setColor(color: Color) {
        LEDPattern.solid(color).applyTo(buffer)
        led.setData(buffer)
    }

    val yellow by command { runOnce { setColor(Color.kYellow) } }

    val blue by command { runOnce { setColor(Color.kBlue) } }

    val red by command { runOnce { setColor(Color.kRed) } }

    val green by command { runOnce { setColor(Color.kGreen) } }
}
