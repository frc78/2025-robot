package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.command

object LEDSubsystem : SubsystemBase() {
    private val led = AddressableLED(0)
    private val buffer = AddressableLEDBuffer(30)

    init {
        led.setColorOrder(AddressableLED.ColorOrder.kRGB)
        LEDPattern.solid(Color.kWhite).applyTo(buffer)
        led.setLength(buffer.length)
        led.setData(buffer)
        led.start()
    }

    val yellow by command {
        runOnce {
            LEDPattern.solid(Color.kYellow).applyTo(buffer)
            led.setData(buffer)
        }
    }

    val blue by command {
        runOnce {
            LEDPattern.solid(Color.kBlue).applyTo(buffer)
            led.setData(buffer)
        }
    }

    val red by command {
        runOnce {
            LEDPattern.solid(Color.kRed).applyTo(buffer)
            led.setData(buffer)
        }
    }

    val green by command {
        runOnce {
            LEDPattern.solid(Color.kGreen).applyTo(buffer)
            led.setData(buffer)
        }
    }
}
