package frc.robot.lib

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches

val Number.degrees
    get() = Degrees.of(this.toDouble())

val Number.inches
    get() = Inches.of(this.toDouble())
