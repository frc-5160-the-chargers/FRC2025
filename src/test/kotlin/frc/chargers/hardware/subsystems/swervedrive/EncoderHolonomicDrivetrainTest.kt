package frc.chargers.hardware.subsystems.swervedrive

import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.framework.UnitTesting
import frc.chargers.utils.epsilonEquals
import frc.chargers.wpilibextensions.Cmd
import frc.chargers.wpilibextensions.angle
import frc.robot.subsystems.getDrivetrain
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

class EncoderHolonomicDrivetrainTest {
    @BeforeEach
    fun setup() = UnitTesting.setup()

    @Test
    fun `angle set should work`() {
        val drivetrain = getDrivetrain()
        UnitTesting.run(
            Cmd.run {
                drivetrain.setTurnDirections(10.degrees, 10.degrees, 10.degrees, 10.degrees)
            }, 2.seconds
        )
        val angles = drivetrain.modulePositions.map { it.angle.angle }
        assert(angles.all { it epsilonEquals 10.degrees }) { "Found angles: ${angles.map { it.inUnit(degrees) }}" }
    }

    @AfterEach
    fun teardown() = UnitTesting.cleanup()
}