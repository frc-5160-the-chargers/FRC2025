package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

/** Stores global state stored accross subsystems. */
public class GlobalState {
    @AutoLogOutput public boolean atL1Range = false;
    public double elevatorVelMPS = 0;
    public boolean hasCoral = false;
}
