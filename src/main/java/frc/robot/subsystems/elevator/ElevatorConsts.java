package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;
import lombok.experimental.FieldDefaults;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Meters;

// Makes everything public and final
@FieldDefaults(makeFinal = true, level = lombok.AccessLevel.PUBLIC)
public class ElevatorConsts {
    private ElevatorConsts() {}

    static double REDUCTION = 5.0;
    static Distance RADIUS = Inches.of(2 * 0.95);
    static Mass CARRIAGE_MASS = Pounds.of(7);
    static DCMotor MOTOR_KIND = DCMotor.getNEO(2);

    static Distance TOLERANCE = Inches.of(0.5);
    static Distance COG_LOW_BOUNDARY = Meters.of(0.6);
    static Distance MAX_HEIGHT = Meters.of(1.285);
    static Distance MIN_HEIGHT = Meters.of(-0.01);
    static double KV = 1 / (MOTOR_KIND.KvRadPerSecPerVolt / REDUCTION * RADIUS.in(Meters));
    // calculate kS by placing robot on its side then running the elevator at tiny voltages until it moves
    // calculate kG by setting voltage until it moves, while upright. Subtract from kS
    // Note: to use calculateWithVelocities(), we need an accurate kA value
    static SimpleMotorFeedforward FEEDFORWARD =
            new SimpleMotorFeedforward(RobotBase.isSimulation() ? 0 : 0.15, KV);
    static double NO_CORAL_KG = 0.43;
    static double WITH_CORAL_KG = 0.5;

    static LinearVelocity MAX_LINEAR_VEL = MetersPerSecond.of((12 - FEEDFORWARD.getKs()) / KV);
    static LinearAcceleration MAX_LINEAR_ACCEL = MetersPerSecondPerSecond.of(6);

    // Leader is the right motor
    static int LEADER_MOTOR_ID = 27;
    static int FOLLOWER_MOTOR_ID = 31;

    static int CURRENT_LIMIT = 80;
    static int SECONDARY_CURRENT_LIMIT = 90;

    static boolean LEADER_INVERTED = true;
    static boolean FOLLOWER_INVERTED = false;
}
