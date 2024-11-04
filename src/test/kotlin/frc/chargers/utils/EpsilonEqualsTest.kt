package frc.chargers.utils

import frc.chargers.framework.UnitTesting
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

class EpsilonEqualsTest {

    @BeforeEach
    fun setUp() = UnitTesting.setup()

    @AfterEach
    fun tearDown() = UnitTesting.cleanup()

    @Test
    fun `epsilonEquals works`() {
        assert((50.999 - 20.0) epsilonEquals 30.999)
        assert((60.0 * 3.9) epsilonEquals 234.0)
        assert((100.0 - 20.0) epsilonEquals 80.0)
    }
}