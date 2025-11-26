// Copyright 2021-2024 FRC 6328
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

package frc.robot.components.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public class Gyro {
    @AutoLog
    public static class GyroData {
        public boolean connected = true;
        public Rotation2d yaw = Rotation2d.kZero;
        public double pitchRad = 0.0;
        public double rollRad = 0.0;
        public double yawVelocityRadPerSec = 0.0;
        public double[] cachedTimestamps = new double[] {};
        public Rotation2d[] cachedYawValues = new Rotation2d[] {};
    }

    public void refreshData(GyroDataAutoLogged inputs) {}
}
