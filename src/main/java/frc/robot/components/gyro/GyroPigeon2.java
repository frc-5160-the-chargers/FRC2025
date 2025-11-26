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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.chargers.hardware.SignalBatchRefresher;
import frc.chargers.misc.Convert;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drive.OdoThread;

import java.util.Queue;

import static frc.robot.subsystems.drive.SwerveConsts.ODO_FREQUENCY_HZ;

/** IO implementation for Pigeon 2. */
public class GyroPigeon2 extends Gyro {
    private final Pigeon2 pigeon =
            new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
    private final BaseStatusSignal
        yaw = pigeon.getYaw(),
        pitch = pigeon.getPitch(),
        roll = pigeon.getRoll(),
        yawVelocity = pigeon.getAngularVelocityZWorld();
    private final Queue<Double>
        yawPositionQueue = OdoThread.getInstance().makeTimestampQueue(),
        timestampQueue = OdoThread.getInstance().register(pigeon.getYaw());

    public GyroPigeon2() {
        var config = TunerConstants.DrivetrainConstants.Pigeon2Configs;
        if (config == null) config = new Pigeon2Configuration();
        pigeon.getConfigurator().apply(config);
        pigeon.getConfigurator().setYaw(0.0);

        yaw.setUpdateFrequency(ODO_FREQUENCY_HZ);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, roll, pitch, yawVelocity);
        SignalBatchRefresher.register(
            TunerConstants.kCANBus.isNetworkFD(),
            roll, pitch, yawVelocity // yaw is already automatically updated by odometry thread
        );
        pigeon.optimizeBusUtilization();
    }

    @Override
    public void refreshData(GyroDataAutoLogged inputs) {
        inputs.connected = BaseStatusSignal.isAllGood(yaw, yawVelocity);
        inputs.yaw = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.rollRad = roll.getValueAsDouble() * Convert.DEGREES_TO_RADIANS;
        inputs.pitchRad = pitch.getValueAsDouble() * Convert.DEGREES_TO_RADIANS;
        inputs.yawVelocityRadPerSec = yawVelocity.getValueAsDouble() * Convert.DEGREES_TO_RADIANS;
        inputs.cachedTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.cachedYawValues =
                yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        timestampQueue.clear();
        yawPositionQueue.clear();
    }
}
