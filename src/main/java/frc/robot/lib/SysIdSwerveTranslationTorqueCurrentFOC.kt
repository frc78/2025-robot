package frc.robot.lib

import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.units.measure.Current

class SysIdSwerveTranslationTorqueCurrentFOC : SwerveRequest {
    private var ampsToApply: Double = 0.0

    /** Local reference to a voltage request for the drive motors */
    private val driveRequest = TorqueCurrentFOC(0.0)

    /** Local reference to a position voltage request for the steer motors */
    private val steerRequestVoltage = PositionVoltage(0.0)

    /** Local reference to a position torque current request for the steer motors */
    private val steerRequestTorqueCurrent = PositionTorqueCurrentFOC(0.0)

    override fun apply(
        parameters: SwerveDrivetrain.SwerveControlParameters,
        vararg modulesToApply: SwerveModule<*, *, *>,
    ): StatusCode? {

        modulesToApply.forEach {
            when (it.steerClosedLoopOutputType) {
                SwerveModuleConstants.ClosedLoopOutputType.Voltage ->
                    it.apply(
                        driveRequest.withOutput(ampsToApply),
                        steerRequestVoltage.withPosition(0.0),
                    )

                SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC ->
                    it.apply(
                        driveRequest.withOutput(ampsToApply),
                        steerRequestTorqueCurrent.withPosition(0.0),
                    )
            }
        }
        return StatusCode.OK
    }

    /**
     * Sets the current to apply to the drive wheels.
     *
     * @param volts Current to apply
     * @return this request
     */
    fun withCurrent(amps: Double) = this.apply { ampsToApply = amps }

    /**
     * Sets the current to apply to the drive wheels.
     *
     * @param amps Current to apply
     * @return this request
     */
    fun withCurrent(amps: Current) = this.apply { ampsToApply = amps.amps }
}
