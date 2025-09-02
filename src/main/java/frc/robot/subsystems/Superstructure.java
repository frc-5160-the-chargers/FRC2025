package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

public class Superstructure {
    public static class SharedState {
        public boolean atL1Range = false;
        public Rotation2d heading = Rotation2d.kZero;
        public double elevatorVelMPS = 0;
        public boolean hasCoral = false;
    }




}
