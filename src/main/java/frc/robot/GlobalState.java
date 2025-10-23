package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

public class GlobalState {
    @AutoLogOutput public boolean atL1Range = false;
    public double elevatorVelMPS = 0;
    public boolean hasCoral = false;
}
