import frc.robot.lib.inches
import frc.robot.lib.radians
import frc.robot.lib.rotations
import frc.robot.lib.toAngle
import kotlin.math.PI
import kotlin.test.Test
import kotlin.test.assertEquals

class UnitsTest {

    @Test
    fun `distance to angle`() {
        var distance = 10.inches
        var radius = 10.inches
        assertEquals(1.radians, distance.toAngle(radius))

        distance = (2 * PI).inches
        radius = 1.inches
        assertEquals(1.0, distance.toAngle(radius).rotations)
    }
}
