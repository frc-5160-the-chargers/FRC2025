package frc.robot.constants;

import frc.robot.subsystems.drive.TunerConstants;
import org.junit.jupiter.api.Test;
import testingutil.ChargerUnitTest;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.*;

class ChoreoVarsTest extends ChargerUnitTest {
    /**
     * Ensures that the robot config specified in choreo lines up with the tuner constants.
     */
    @Test
    public void ensureConsistencyWithTunerConstants() {
        assertEquals(
            TunerConstants.FrontLeft.DriveMotorGearRatio,
            ChoreoVars.gearing,
            0.001,
            "Incorrect gearing in choreo."
        );
        assertEquals(
            TunerConstants.FrontLeft.WheelRadius,
            ChoreoVars.wheelRadius.in(Meters),
            0.001,
            "Incorrect moduleY in choreo."
        );
        assertEquals(
            TunerConstants.FrontLeft.LocationX,
            ChoreoVars.moduleX.in(Meters),
            0.001,
            "Incorrect moduleX in choreo."
        );
        assertEquals(
            TunerConstants.FrontLeft.LocationY,
            ChoreoVars.moduleY.in(Meters),
            0.001,
            "Incorrect moduleY in choreo."
        );
    }
}