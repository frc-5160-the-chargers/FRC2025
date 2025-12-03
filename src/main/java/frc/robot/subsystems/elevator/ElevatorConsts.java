package frc.robot.subsystems.elevator;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;
import frc.chargers.misc.TunableValues.Tunable;
import frc.chargers.misc.TunableValues.TunableNum;

import static edu.wpi.first.units.Units.*;

public class ElevatorConsts {
    private ElevatorConsts() {}

    static final Tunable<Distance>
        DEMO_HEIGHT = new Tunable<>("Elevator/DemoHeight", Meters.of(0.5)),
        TOLERANCE = new Tunable<>("Elevator/Tolerance", Inches.of(0.5)),
        COG_LOW_BOUNDARY = new Tunable<>("Elevator/CoGLowBoundary", Meters.of(0.6));
    static final Tunable<LinearAcceleration> MAX_ACCEL =
        new Tunable<>("Elevator/MaxAccel", MetersPerSecondPerSecond.of(6));
    static final TunableNum DEMO_VOLTS = new TunableNum("Elevator/DemoVolts", 3);

    static final double REDUCTION = 5.0;
    static final Mass CARRIAGE_MASS = Pounds.of(9);
    static final Distance RADIUS = Inches.of(2 * 0.95);
    static final DCMotor MOTOR_KIND = DCMotor.getNEO(2);
    static final LinearSystem<N2, N1, N2> ELEVATOR_SYSTEM =
        LinearSystemId.createElevatorSystem(
            MOTOR_KIND, CARRIAGE_MASS.in(Kilograms),
            RADIUS.in(Meters), REDUCTION
        );

    static final Distance MAX_HEIGHT = Meters.of(1.285);
    static final Distance MIN_HEIGHT = Meters.of(-0.01);
    static final double KP = 1.0;
    static final double KD = 0.01;
    static final double KV = 1 / (MOTOR_KIND.KvRadPerSecPerVolt / REDUCTION * RADIUS.in(Meters));
    // calculate kS(volts) by placing robot on its side then running the elevator at tiny voltages until it moves
    // calculate kG by setting voltage until it moves, while upright. Subtract from kS
    // Note: to use calculateWithVelocities(), we need an accurate kA value
    static final double KS = RobotBase.isSimulation() ? 0 : 0.15;

    static final LinearVelocity MAX_LINEAR_VEL = MetersPerSecond.of((12 - KS) / KV);

    // Leader is the right motor
    static final int LEADER_MOTOR_ID = 27;
    static final int FOLLOWER_MOTOR_ID = 31;

    static final int CURRENT_LIMIT = 80;
    static final int SECONDARY_CURRENT_LIMIT = 90;

    static final boolean INVERTED = true;
}