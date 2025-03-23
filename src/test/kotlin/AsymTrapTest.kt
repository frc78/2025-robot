import frc.robot.lib.AsymtrapezoidalProfile
import java.io.FileOutputStream
import java.io.OutputStream
import kotlin.test.Test

class AsymTrapTest {
    val profile = AsymtrapezoidalProfile(AsymtrapezoidalProfile.Constraints(2.0, 1.0, 0.5))
    val goal = AsymtrapezoidalProfile.State(10.0, 0.0)
    var current = AsymtrapezoidalProfile.State(0.0, 1.0)

    @Test
    fun testProfile() {
        FileOutputStream("AsymTestOutput.csv").apply { writeCsv() }
    }

    fun OutputStream.writeCsv() {
        val writer = bufferedWriter()
        writer.write("t, position, vel")
        writer.newLine()
        for (i in 0 until 500) {

            current = profile.calculate(0.02, current, goal)
            writer.write("${i / 50.0}, ${current.position}, ${current.velocity}")
            writer.newLine()
        }
        writer.flush()
    }
}
