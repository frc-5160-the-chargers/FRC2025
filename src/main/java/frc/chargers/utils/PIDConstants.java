package frc.chargers.utils;

public class PIDConstants {
	public final double kP;
	public final double kI;
	public final double kD;
	
	public static final PIDConstants VOID = new PIDConstants(0, 0, 0);
	
	public PIDConstants(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}
}
