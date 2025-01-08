package frc.chargers.utils;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.Units;

public class ShorterUnitsNames {
	private ShorterUnitsNames() {}
	
	public static final MomentOfInertiaUnit KgSquareMeters = Units.KilogramSquareMeters;
	public static final MomentOfInertiaUnit KgMetersSquared = KgSquareMeters;
	public static final LinearAccelerationUnit MetersPerSecSquared = Units.MetersPerSecondPerSecond;
	public static final AngularAccelerationUnit RadiansPerSecSquared = Units.RadiansPerSecondPerSecond;
	
	public static final LinearVelocityUnit MetersPerSec = Units.MetersPerSecond;
	public static final AngularVelocityUnit RadiansPerSec = Units.RadiansPerSecond;
}
