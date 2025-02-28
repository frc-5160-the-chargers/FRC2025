package frc.robot;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.GrappleJNI;
import edu.wpi.first.wpilibj.TimedRobot;

public class LaserCANPatch extends TimedRobot {
	public LaserCANPatch() {
		GrappleJNI.forceLoad();
		CanBridge.runTCP();
	}
}
