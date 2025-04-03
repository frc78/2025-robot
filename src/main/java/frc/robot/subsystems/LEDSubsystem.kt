package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.Level
import frc.robot.lib.ScoreSelector
import frc.robot.lib.command
import frc.robot.lib.seconds

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

    fun setColorForSelectedLevel() = setColor(colorForSelectedLevel)

    private val colorForSelectedLevel
        get() = when (ScoreSelector.SelectedLevel) {
            Level.L1 -> Color.kGreen
            Level.L2 -> Color.kRed
            Level.L3 ->Color.kBlue
            Level.L4 -> Color.kYellow
        }

    val flashForSelectedLevel by command {
        Commands.run ({
            val base = LEDPattern.solid(colorForSelectedLevel)
            val pattern = base.blink(.25.seconds)
            pattern.applyTo(buffer)
            led.setData(buffer)
        })
    }
    private fun setColor(color: Color) {
        LEDPattern.solid(color).applyTo(buffer)
        led.setData(buffer)
    }

    val yellow by command { runOnce { setColor(Color.kYellow) } }

    val blue by command { runOnce { setColor(Color.kBlue) } }

    val red by command { runOnce { setColor(Color.kRed) } }

    private fun flashColor(color: Color): Command {
        val base = LEDPattern.solid(color)
        val pattern = base.blink(.25.seconds)
        return runOnce {
            pattern.applyTo(buffer)
            led.setData(buffer)
        }
    }
     val flashWhite by command {
         flashColor(Color.kWhite)
     }
    val flashPink by command {
        flashColor(Color.kHotPink)
    }
    val flashYellow by command {
        flashColor(Color.kYellow)
    }
    val flashRed by command {
        flashColor(Color.kRed)
    }
    val flashBlue by command {
        flashColor(Color.kBlue)
    }
    val flashGreen by command {
        flashColor(Color.kGreen)
    }

    val green by command { runOnce { setColor(Color.kGreen) } }
}
