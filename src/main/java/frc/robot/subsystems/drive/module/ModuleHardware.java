package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.chargers.hardware.MotorDataAutoLogged;
import org.littletonrobotics.junction.AutoLog;

public class ModuleHardware {
    @AutoLog
    public static class ModuleData {
        public MotorDataAutoLogged steer = new MotorDataAutoLogged();
        public MotorDataAutoLogged drive = new MotorDataAutoLogged();
        public Rotation2d steerAbsolutePos = new Rotation2d();

        public double[] odoTimestamps = {};
        public double[] odoDrivePositionsRad = {};
        public Rotation2d[] odoSteerPositions = {};
    }

    /** Updates the data related to the swerve module hardware. */
    public void refreshData(ModuleDataAutoLogged inputs) {}

    /** Run the drive motor at the specified open loop value. */
    public void setDriveOpenLoop(double voltsOrAmps) {}

    /** Run the turn motor at the specified open loop value. */
    public void setSteerOpenLoop(double voltsOrAmps) {}

    /** Run the drive motor at the specified velocity. */
    public void setDriveVelocity(double radPerSec) {}

    /** Run the turn motor to the specified rotation. */
    public void setSteerPosition(Rotation2d rotation) {}
}
