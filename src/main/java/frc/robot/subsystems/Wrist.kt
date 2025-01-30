package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.Subsystem

<<<<<<< HEAD
object Wrist : Subsystem {}
=======
object Wrist : Subsystem {
    fun goTo(state: RobotState) = PrintCommand("Wrist going to $state - ${state.wristAngle}")
}
>>>>>>> 697c837 (RobotoStates)
