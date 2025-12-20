package frc.robot.subsystems

import frc.robot.subsystems.drivetrain.Chassis

object StateMachineManager {
    fun robotInit() {}

    fun robotPeriodic() {
        SuperStructure.periodic()
        Intake.periodic()
        Climber.periodic()
        Chassis.periodic()
    }

    fun autonomousInit() {}

    fun autonomousPeriodic() {
        SuperStructure.stateMachine()
        Intake.stateMachine()
        LEDSubsystem.stateMachine()
    }

    fun teleopInit() {}

    fun teleopPeriodic() {
        SuperStructure.stateMachine()
        Intake.stateMachine()
        LEDSubsystem.stateMachine()
        Climber.stateMachine()
        Chassis.stateMachine()
    }

    fun disabledInit() {}

    fun disabledPeriodic() {}

    fun simulationInit() {}

    fun simulationPeriodic() {
        SuperStructure.simulationPeriodic()
        Intake.simulationPeriodic()
    }
}
