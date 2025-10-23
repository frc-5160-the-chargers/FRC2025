package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.chargers.misc.TunableValues.TunableNum;

import static edu.wpi.first.units.Units.*;

public class WristConsts {
    static final TunableNum
        KP = new TunableNum("coralIntakePivot/kP", 0.64),
        KD = new TunableNum("coralIntakePivot/kD", 0.01),
        DEMO_TARGET_DEG = new TunableNum("coralIntakePivot/demoTarget(deg)", 0),
        DEMO_VOLTS = new TunableNum("coralIntakePivot/demoVolts", 0);

    static final DCMotor MOTOR_KIND = DCMotor.getNeo550(1);
    static final int MOTOR_ID = 13;
    static final Angle TOLERANCE = Degrees.of(1);
    static final double REDUCTION = 72;
    static final MomentOfInertia MOI = KilogramSquareMeters.of(0.012);
    static final Angle ZERO_OFFSET = Radians.of(1.7);

    static final double
        KS = 0.05,
        KV = 1 / (MOTOR_KIND.KvRadPerSecPerVolt / REDUCTION);
    static final Voltage
        NO_CORAL_KG = Volts.of(-0.32),
        WITH_CORAL_KG = Volts.of(-0.49);
    static final double
        MAX_VEL = (12 - KS) / KV, // rad/sec
        MAX_ACCEL = 40; // rad/sec^2
    static final double CURRENT_LIMIT = 60;
}
