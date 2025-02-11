// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.chargers.utils.TunableValues.TunableNum;
import frc.robot.subsystems.swerve.SwerveDrive;
import lombok.RequiredArgsConstructor;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Meters;
import static monologue.Monologue.GlobalLog;

public class WheelRadiusCharacterization extends Command {
	private static final TunableNum characterizationSpeed =
		new TunableNum("WheelRadiusCharacterization/SpeedRadsPerSec", 0.1);
	
	@RequiredArgsConstructor
	public enum Direction {
		CLOCKWISE(-1),
		COUNTER_CLOCKWISE(1);
		
		private final int value;
	}
	
	private final SwerveDrive drive;
	private final Direction omegaDirection;
	private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);
	private final DoubleSupplier gyroYawRadsSupplier;
	private final Distance driveBaseRadius;
	
	private double lastGyroYawRads = 0.0;
	private double accumGyroYawRads = 0.0;
	
	private double[] startWheelPositions;
	
	private double currentEffectiveWheelRadius = 0.0;
	
	public WheelRadiusCharacterization(SwerveDrive drive, Direction omegaDirection) {
		this.drive = drive;
		this.omegaDirection = omegaDirection;
		this.gyroYawRadsSupplier = () -> drive.poseEstimate().getRotation().getRadians();
		this.driveBaseRadius = drive.config.ofHardware().drivebaseRadius();
		addRequirements(drive);
	}
	
	private double[] getWheelPositions() {
		return Arrays.stream(drive.swerveModules).mapToDouble(it -> it.currentPosition().distanceMeters).toArray();
	}
	
	@Override
	public void initialize() {
		// Reset
		lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
		accumGyroYawRads = 0.0;
		startWheelPositions = getWheelPositions();
		omegaLimiter.reset(0);
	}
	
	@Override
	public void execute() {
		// Run drive at velocity
		var rotationSpeed = omegaLimiter.calculate(omegaDirection.value * characterizationSpeed.get());
		var moduleStates = drive.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, rotationSpeed));
		for (int i = 0; i < 4; i++) {
			drive.swerveModules[i].setDesiredState(moduleStates[i], true, 0);
		}
		
		// Get yaw and wheel positions
		accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
		lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
		double averageWheelPosition = 0.0;
		double[] wheelPositions = getWheelPositions();
		for (int i = 0; i < 4; i++) {
			averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
		}
		averageWheelPosition /= 4.0;
		
		currentEffectiveWheelRadius = (accumGyroYawRads * driveBaseRadius.in(Meters)) / averageWheelPosition;
		GlobalLog.log("WheelRadiusCharacterization/DrivePosition", averageWheelPosition);
		GlobalLog.log("WheelRadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
		GlobalLog.log("WheelRadiusCharacterization/CurrentWheelRadiusInches", Units.metersToInches(currentEffectiveWheelRadius));
	}
	
	@Override
	public void end(boolean interrupted) {
		if (accumGyroYawRads <= Math.PI * 2.0) {
			System.out.println("Not enough data for characterization");
		} else {
			System.out.println(
				"Effective Wheel Radius: "
					+ Units.metersToInches(currentEffectiveWheelRadius)
					+ " inches");
		}
	}
}