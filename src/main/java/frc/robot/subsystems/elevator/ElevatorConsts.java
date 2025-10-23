package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;
import frc.chargers.misc.TunableValues.TunableNum;

import static edu.wpi.first.units.Units.*;

public class ElevatorConsts {
    private ElevatorConsts() {}

    static final TunableNum
        DEMO_HEIGHT = new TunableNum("Elevator/DemoHeightMeters", 0.5),
        DEMO_VOLTS = new TunableNum("Elevator/DemoVolts", 3),
        KP = new TunableNum("Elevator/KP", 0.5),
        KD = new TunableNum("Elevator/KD", 0);

    static final double REDUCTION = 5.0;
    static final Distance RADIUS = Inches.of(2 * 0.95);
    static final Mass CARRIAGE_MASS = Pounds.of(7);
    static final DCMotor MOTOR_KIND = DCMotor.getNEO(2);

    static final Distance TOLERANCE = Inches.of(0.5);
    static final Distance COG_LOW_BOUNDARY = Meters.of(0.6);
    static final Distance MAX_HEIGHT = Meters.of(1.285);
    static final Distance MIN_HEIGHT = Meters.of(-0.01);
    // Volts / (Meters / Seconds)
    static final double KV = 1 / (MOTOR_KIND.KvRadPerSecPerVolt / REDUCTION * RADIUS.in(Meters));
    // calculate kS(volts) by placing robot on its side then running the elevator at tiny voltages until it moves
    // calculate kG by setting voltage until it moves, while upright. Subtract from kS
    // Note: to use calculateWithVelocities(), we need an accurate kA value
    static final double KS = RobotBase.isSimulation() ? 0 : 0.15;
    static final double NO_CORAL_KG = 0.43;
    static final double WITH_CORAL_KG = 0.5;

    static final LinearVelocity MAX_LINEAR_VEL = MetersPerSecond.of((12 - KS) / KV);
    static final LinearAcceleration MAX_LINEAR_ACCEL = MetersPerSecondPerSecond.of(6);

    // Leader is the right motor
    static final int LEADER_MOTOR_ID = 27;
    static final int FOLLOWER_MOTOR_ID = 31;

    static final int CURRENT_LIMIT = 80;
    static final int SECONDARY_CURRENT_LIMIT = 90;

    static final boolean INVERTED = true;
}
