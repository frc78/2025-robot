package frc.robot.lib

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Centimeters
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Kilograms
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.MetersPerSecondPerSecond
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.RadiansPerSecondPerSecond
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.units.measure.Voltage

/* Extension properties are ways of adding functionality to classes
 * we didn't create, without having to create a subclass.*/

// These extension properties add units to numbers to make more readable code
// You can use them like so:
// val length = 5.inches
// val angle = 10.degrees
val Number.inches: Distance
    get() = Inches.of(this.toDouble())
val Number.meters: Distance
    get() = Meters.of(this.toDouble())
val Number.centimeters: Distance
    get() = Centimeters.of(this.toDouble())
val Number.seconds: Time
    get() = Seconds.of(this.toDouble())
val Number.metersPerSecond: LinearVelocity
    get() = MetersPerSecond.of(this.toDouble())
val Number.volts: Voltage
    get() = Volts.of(this.toDouble())
val Number.amps: Current
    get() = Amps.of(this.toDouble())
val Number.metersPerSecondPerSecond: LinearAcceleration
    get() = MetersPerSecondPerSecond.of(this.toDouble())
val Number.radiansPerSecond: AngularVelocity
    get() = RadiansPerSecond.of(this.toDouble())
val Number.radiansPerSecondPerSecond: AngularAcceleration
    get() = RadiansPerSecondPerSecond.of(this.toDouble())
val Number.rpm
    get() = RPM.of(this.toDouble())
val Number.degrees: Angle
    get() = Degrees.of(this.toDouble())
val Number.rotationsPerSecond: AngularVelocity
    get() = RotationsPerSecond.of(this.toDouble())
val Number.voltsPerSecond: Velocity<VoltageUnit>
    get() = this.volts.per(Second)

// These extension properties allow converting from a unit to a raw value
// You can use them like so:
// val meters = Meters.of(10)
// val tenMetersInInches = meters.inches
// val rotations = Rotations.of(5)
// val fiveRotationsInDegrees = rotations.degrees
val Distance.inches
    get() = this.`in`(Inches)
val Distance.meters
    get() = this.`in`(Meters)
val Distance.centimeters
    get() = this.`in`(Centimeters)
val Angle.rotations
    get() = this.`in`(Rotations)
val LinearVelocity.metersPerSecond
    get() = this.`in`(MetersPerSecond)
val AngularVelocity.radiansPerSecond
    get() = this.`in`(RadiansPerSecond)
val AngularVelocity.rotationsPerSecond
    get() = this.`in`(RotationsPerSecond)
val AngularVelocity.rpm
    get() = this.`in`(RPM)
val Voltage.volts
    get() = this.`in`(Volts)
val Angle.radians
    get() = this.`in`(Radians)
val Angle.degrees
    get() = this.`in`(Degrees)
val Current.amps
    get() = this.`in`(Amps)
val Mass.kilograms
    get() = this.`in`(Kilograms)

fun Angle.toDistance(radius: Distance): Distance = radius * this.radians

fun AngularVelocity.toLinearVelocity(radius: Distance): LinearVelocity =
    MetersPerSecond.of(radiansPerSecond * radius.meters)
