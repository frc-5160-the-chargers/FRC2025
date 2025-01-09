package frc.chargers.utils;

import lombok.AllArgsConstructor;

@AllArgsConstructor
public class PIDConstants {
	public final double kP;
	public final double kI;
	public final double kD;
	
	public static final PIDConstants VOID = new PIDConstants(0, 0, 0);
}
