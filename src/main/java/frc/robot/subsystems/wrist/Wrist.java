package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chargers.data.InputStream;
import frc.chargers.data.TunableValues.TunableNum;
import frc.chargers.hardware.MotorInputsAutoLogged;
import frc.chargers.utils.RobotMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure.SharedData;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.wrist.WristConsts.*;

public class Wrist extends SubsystemBase {
    private final TunableNum kP = new TunableNum("coralIntakePivot/kP", 0.64);
    private final TunableNum kD = new TunableNum("coralIntakePivot/kD", 0.01);
    private final TunableNum demoTargetDeg = new TunableNum("coralIntakePivot/demoTarget(deg)", 0);
    private final TunableNum demoVolts = new TunableNum("coralIntakePivot/demoVolts", 0);

    private final WristHardware io;
    private final SharedData sharedData;
    private final TrapezoidProfile motionProfile = new TrapezoidProfile(new Constraints(MAX_VEL, MAX_ACCEL));
    private final MotorInputsAutoLogged inputData = new MotorInputsAutoLogged();

    @AutoLogOutput private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
    @AutoLogOutput private Angle target = Degrees.of(Double.NaN);
    @AutoLogOutput private double feedforwardV = 0.0;

    @AutoLogOutput
    public final Trigger atTarget =
        new Trigger(() -> Math.abs(inputData.positionRad - target.in(Radians)) < TOLERANCE.in(Radians));

    public Wrist(SharedData sharedData) {
        this.sharedData = sharedData;
        this.io = switch (RobotMode.get()) {
            case REAL -> new RealWristHardware();
            case SIM -> new SimWristHardware();
            case REPLAY -> new WristHardware();
        };
        io.setPDGains(kP.get(), kD.get());
        kP.onChange(p -> io.setPDGains(p, kD.get()));
        kD.onChange(d -> io.setPDGains(kP.get(), d));
        setDefaultCommand(idleCmd());
    }

    @AutoLogOutput
    public double gravityCompensationV() {
        if (RobotBase.isSimulation()) return 0.0;
        var kg = sharedData.hasCoral ? WITH_CORAL_KG : NO_CORAL_KG;
        return kg.in(Volts) * Math.cos(inputData.positionRad);
    }

    public Command idleCmd() {
        return this.run(() -> io.setVolts(gravityCompensationV()));
    }

    public Command setDemoAngleCmd() {
        return Commands.defer(() -> setAngleCmd(Degrees.of(demoTargetDeg.get())), Set.of(this));
    }

    public Command setAngleCmd(Angle target) {
        var goalState = new TrapezoidProfile.State(target.in(Radians), 0);
        return Commands.runOnce(() -> {
            currentSetpoint = new TrapezoidProfile.State(
                inputData.positionRad, inputData.velocityRadPerSec
            );
            this.target = target;
        })
            .andThen(
                this.run(() -> {
                    currentSetpoint = motionProfile.calculate(0.02, currentSetpoint, goalState);
                    feedforwardV = FF_EQUATION.calculate(currentSetpoint.velocity);
                    feedforwardV += gravityCompensationV();
                    io.setRadians(currentSetpoint.position, feedforwardV);
                })
            )
            .until(atTarget.and(() -> Math.abs(sharedData.elevatorVelMPS) < 0.08))
            .withName("set angle (pivot)");
    }

    public Command setPowerCmd(InputStream controllerInput) {
        return this.run(() -> io.setVolts(controllerInput.get() * 12 + gravityCompensationV()))
            .withName("set power(pivot)");
    }

    public Command setDemoVoltageCmd() {
        return this.run(() -> io.setVolts(demoVolts.get()))
            .withName("set demo volts(pivot)");
    }

    @Override
    public void periodic() {
        io.refreshData(inputData);
        Logger.processInputs("Wrist", inputData);
    }
}
