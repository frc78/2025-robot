package frc.robot.lib

import com.ctre.phoenix6.Utils
import edu.wpi.first.units.AngularAccelerationUnit
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj.Timer

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
val Number.feetPerSecond: LinearVelocity
    get() = FeetPerSecond.of(this.toDouble())
val Number.volts: Voltage
    get() = Volts.of(this.toDouble())
val Number.amps: Current
    get() = Amps.of(this.toDouble())
val Number.metersPerSecondPerSecond: LinearAcceleration
    get() = MetersPerSecondPerSecond.of(this.toDouble())
val Number.radians: Angle
    get() = Radians.of(this.toDouble())
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
val Number.rotationsPerSecondPerSecond: AngularAcceleration
    get() = RotationsPerSecondPerSecond.of(this.toDouble())
val Number.rotationsPerSecondCubed: Velocity<AngularAccelerationUnit>
    get() = RotationsPerSecondPerSecond.of(this.toDouble()).per(Second)
val Number.voltsPerSecond: Velocity<VoltageUnit>
    get() = Volts.of(this.toDouble()).per(Second)
val Number.rotations: Angle
    get() = Rotations.of(this.toDouble())
val Number.pounds: Mass
    get() = Pounds.of(this.toDouble())
val Number.kilogramSquareMeters: MomentOfInertia
    get() = KilogramSquareMeters.of(this.toDouble())

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
val LinearVelocity.metersPerSecond
    get() = this.`in`(MetersPerSecond)
val AngularVelocity.radiansPerSecond
    get() = this.`in`(RadiansPerSecond)
val AngularVelocity.rotationsPerSecond
    get() = this.`in`(RotationsPerSecond)
val AngularVelocity.rpm
    get() = this.`in`(RPM)
val AngularAcceleration.radiansPerSecondPerSecond
    get() = this.`in`(RadiansPerSecondPerSecond)
val Voltage.volts
    get() = this.`in`(Volts)
val Angle.radians
    get() = this.`in`(Radians)
val Angle.degrees
    get() = this.`in`(Degrees)
val Angle.rotations
    get() = this.`in`(Rotations)
val Current.amps
    get() = this.`in`(Amps)
val Mass.kilograms
    get() = this.`in`(Kilograms)

fun Angle.toDistance(radius: Distance): Distance = radius * this.radians

fun Distance.toAngle(radius: Distance): Angle = Radians.of((this / radius).magnitude())

fun LinearVelocity.toAngularVelocity(radius: Distance): AngularVelocity =
    RadiansPerSecond.of(this.metersPerSecond / radius.meters)

fun AngularVelocity.toLinearVelocity(radius: Distance): LinearVelocity =
    MetersPerSecond.of(radiansPerSecond * radius.meters)

// fun fpgaToCurrentTime(fpgaTimeSeconds: Double): Double {
//    return (Utils.getCurrentTimeSeconds() - Timer.getFPGATimestamp()) + fpgaTimeSeconds
// }

fun currentTimeToFPGA(currentTime: Double): Double {
    return currentTime - (Utils.getCurrentTimeSeconds() - Timer.getFPGATimestamp())
}
