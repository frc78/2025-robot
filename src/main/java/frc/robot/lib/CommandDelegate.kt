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
 * With the normal method of using a factory function, it is easy to make the mistake of calling the
 * factory function as a normal function, expecting it to do something to the mechanism.
 *
 * Since commands are more akin to properties of a subsystem, it makes sense to make them a
 * property, however they still need to be created each time they're accessed. This delegate allows
 * for that.
 *
 * While it is not necessary to use this delegate, it is highly recommended ot avoid common pitfalls
 * and make reviewing code easier.
 *
 * To use this delegate, simply declare a command like so:
 *
 * val subsystemCommand by command { runOnce { /* code */ } }
 */
class CommandDelegate(private val command: () -> Command) : ReadOnlyProperty<Any?, Command> {
    override fun getValue(thisRef: Any?, property: KProperty<*>): Command {
        return command()
    }
}

fun command(command: () -> Command) = CommandDelegate(command)
