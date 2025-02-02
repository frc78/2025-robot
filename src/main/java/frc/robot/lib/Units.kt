package frc.robot.lib

import edu.wpi.first.units.Units.Centimeters
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance

/* Extension properties are ways of adding functionality to classes
 * we didn't create, without having to create a subclass.*/

// These extension properties add units to numbers to make more readable code
// You can use them like so:
// val length = 5.inches
// val angle = 10.degrees
val Number.degrees: Angle
    get() = Degrees.of(this.toDouble())

val Number.inches: Distance
    get() = Inches.of(this.toDouble())
val Number.centimeters: Distance
    get() = Centimeters.of(this.toDouble())

// These extension properties allow converting from a unit to a raw value
// You can use them like so:
// val meters = Meters.of(10)
// val tenMetersInInches = meters.inches
// val rotations = Rotations.of(5)
// val fiveRotationsInDegrees = rotations.degrees
val Distance.inches
    get() = this.`in`(Inches)
val Distance.centimeters
    get() = this.`in`(Centimeters)
val Angle.degrees
    get() = this.`in`(Degrees)
