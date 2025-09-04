package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import lombok.AccessLevel;
import lombok.experimental.FieldDefaults;

import static edu.wpi.first.units.Units.*;

@FieldDefaults(level = AccessLevel.PUBLIC, makeFinal = true)
public class WristConsts {
    static DCMotor MOTOR_KIND = DCMotor.getNeo550(1);
    static int MOTOR_ID = 13;
    static Angle TOLERANCE = Degrees.of(1);
    static double REDUCTION = 72;
    static MomentOfInertia MOI = KilogramSquareMeters.of(0.012);
    static Angle ZERO_OFFSET = Radians.of(1.7);

    static double KV = 1 / (MOTOR_KIND.KvRadPerSecPerVolt / REDUCTION);
    static SimpleMotorFeedforward FF_EQUATION = new SimpleMotorFeedforward(0.05, KV);
    static Voltage NO_CORAL_KG = Volts.of(-0.32);
    static Voltage WITH_CORAL_KG = Volts.of(-0.49);

    static double MAX_VEL = (12 - FF_EQUATION.getKs()) / KV; // rad/sec
    static double MAX_ACCEL = 40; // rad/sec^2

    static double CURRENT_LIMIT = 60;
}
