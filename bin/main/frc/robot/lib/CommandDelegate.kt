package frc.robot.lib

import edu.wpi.first.wpilibj2.command.Command
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty

/**
 * Delegate for creating commands.
 *
 * This delegate class is more of a teaching tool and used to ensure that any command is created via
 * a factory function, and not a single instance. Students can guarantee that a command is created
 * properly by seeing the 'by command' phrase in each command declaration.
 *
 * While it is not necessary to use this delegate, it is highly recommended ot avoid common pitfalls
 * and make reviewing code easier.
 */
class CommandDelegate(private val command: () -> Command) : ReadOnlyProperty<Any?, Command> {
    override fun getValue(thisRef: Any?, property: KProperty<*>): Command {
        return command()
    }
}

fun command(command: () -> Command) = CommandDelegate(command)
