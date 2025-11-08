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

package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.chargers.data.RobotMode;
import lombok.Getter;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.Logger;

import static frc.chargers.commands.TriggerUtil.bind;

public class SwerveModule {
    private final ModuleHardware io;
    private final ModuleDataAutoLogged inputs = new ModuleDataAutoLogged();
    private final String name;
    private final SwerveModuleConstants<?,?,?> constants;

    /**
     * Fetches an array of many intermediately-recorded SwerveModulePositions,
     * that can be fed to a pose estimation algorithm.
     */
    @Getter private SwerveModulePosition[] odometryFrames = {};

    public SwerveModule(
        String name,
        SwerveModuleSimulation sim,
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants
    ) {
        this.name = name;
        this.io = switch (RobotMode.get()) {
            case REAL -> new RealModuleHardware(constants);
            case SIM -> new SimModuleHardware(constants, sim);
            case REPLAY -> new ModuleHardware();
        };
        this.constants = constants;
        bind(
            new Alert("Drive Motor Error on module " + name + ".", AlertType.kError),
            () -> !inputs.drive.ok()
        );
        bind(
            new Alert("Turn Motor Error on module " + name + ".", AlertType.kError),
            () -> !inputs.steer.ok()
        );
    }

    public void periodic() {
        io.refreshData(inputs);
        Logger.processInputs("SwerveDrive/Modules/" + name, inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odoTimestamps.length; // All signals are sampled together
        odometryFrames = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            odometryFrames[i] = new SwerveModulePosition(
                inputs.odoDrivePositionsRad[i] * constants.WheelRadius,
                inputs.odoSteerPositions[i]
            );
        }
    }

    /** For rickrolling purposes */
    public void addInstruments(Orchestra orchestra) {
        io.addInstruments(orchestra);
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    public void runSetpoint(SwerveModuleState state) {
        io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
        io.setSteerPosition(state.angle);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void driveStraight(double output) {
        io.setDriveOpenLoop(output);
        io.setSteerPosition(new Rotation2d());
    }

    /** Spins the azimuth motor at a certain output while keeping the drive motor still */
    public void spinAzimuth(double output) {
        io.setDriveOpenLoop(0);
        io.setSteerOpenLoop(output);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoop(0);
        io.setSteerOpenLoop(0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.steerAbsolutePos;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drive.positionRad * constants.WheelRadius,
            getAngle()
        );
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.drive.velocityRadPerSec * constants.WheelRadius,
            getAngle()
        );
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdoTimestamps() {
        return inputs.odoTimestamps;
    }

    /** Fetches the raw angular position of the drive motor. */
    public double getDistTraveledAngular() {
        return inputs.drive.positionRad;
    }
}
