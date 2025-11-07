// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.Orchestra;
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
    public void setDriveOpenLoop(double output) {}

    /** Run the turn motor at the specified open loop value. */
    public void setSteerOpenLoop(double output) {}

    /** Run the drive motor at the specified velocity. */
    public void setDriveVelocity(double velocityRadPerSec) {}

    /** Run the turn motor to the specified rotation. */
    public void setSteerPosition(Rotation2d rotation) {}

    /** For rick rolling ppl */
    public void addInstruments(Orchestra orchestra) {}
}
