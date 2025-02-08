package frc.robot.lib

import edu.wpi.first.units.Units.Centimeters
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.units.measure.Voltage

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
val Number.metersPerSecond: LinearVelocity
    get() = MetersPerSecond.of(this.toDouble())
val Number.rotationsPerSecond: AngularVelocity
    get() = RotationsPerSecond.of(this.toDouble())
val Number.volts: Voltage
    get() = Volts.of(this.toDouble())
val Number.voltsPerSecond: Velocity<VoltageUnit>
    get() = Volts.of(this.toDouble()).per(Second)

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
val LinearVelocity.metersPerSecond
    get() = this.`in`(MetersPerSecond)
val AngularVelocity.radiansPerSecond
    get() = this.`in`(RadiansPerSecond)
val Voltage.volts
    get() = this.`in`(Volts)
