package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chargers.data.InputStream;
import frc.chargers.hardware.MotorDataAutoLogged;
import frc.chargers.data.RobotMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GlobalState;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.wrist.WristConsts.*;

public class Wrist extends SubsystemBase {
    private final WristHardware io;
    private final GlobalState globalState;
    private final TrapezoidProfile motionProfile = new TrapezoidProfile(new Constraints(MAX_VEL, MAX_ACCEL));
    @Getter private final MotorDataAutoLogged inputs = new MotorDataAutoLogged();

    private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
    @AutoLogOutput private Angle target = Degrees.of(Double.NaN);
    @AutoLogOutput private double feedforwardV = 0.0;

    @AutoLogOutput
    public final Trigger
        atTarget = new Trigger(() -> Math.abs(inputs.positionRad - target.in(Radians)) < TOLERANCE.in(Radians));

    public Wrist(GlobalState globalState) {
        this.globalState = globalState;
        this.io = switch (RobotMode.get()) {
            case REAL -> new RealWristHardware();
            case SIM -> new SimWristHardware();
            case REPLAY -> new WristHardware();
        };
        io.setPDGains(KP.get(), KD.get());
        KP.onChange(p -> io.setPDGains(p, KD.get()));
        KD.onChange(d -> io.setPDGains(KP.get(), d));
    }

    @AutoLogOutput
    public double gravityCompensationV() {
        if (RobotBase.isSimulation()) return 0.0;
        var kg = globalState.hasCoral ? WITH_CORAL_KG : NO_CORAL_KG;
        return kg.in(Volts) * Math.cos(inputs.positionRad);
    }

    public Command idleCmd() {
        return this.run(() -> io.setVolts(gravityCompensationV()));
    }

    public Command setDemoAngleCmd() {
        return Commands.defer(() -> setAngleCmd(Degrees.of(DEMO_TARGET_DEG.get())), Set.of(this));
    }

    public Command setAngleCmd(Angle target) {
        var goalState = new TrapezoidProfile.State(target.in(Radians), 0);
        return Commands.runOnce(() -> {
            currentSetpoint = new TrapezoidProfile.State(
                inputs.positionRad, inputs.velocityRadPerSec
            );
            this.target = target;
        })
            .andThen(
                this.run(() -> {
                    currentSetpoint = motionProfile.calculate(0.02, currentSetpoint, goalState);
                    feedforwardV = Math.signum(currentSetpoint.velocity) * KS
                        + currentSetpoint.velocity * KV
                        + gravityCompensationV();
                    io.setRadians(currentSetpoint.position, feedforwardV);
                })
            )
            .until(atTarget.and(() -> Math.abs(globalState.elevatorVelMPS) < 0.08))
            .withName("set angle (pivot)");
    }

    public Command setPowerCmd(InputStream controllerInput) {
        return this.run(() -> io.setVolts(controllerInput.get() * 12 + gravityCompensationV()))
            .withName("set power(pivot)");
    }

    public Command setDemoVoltageCmd() {
        return this.run(() -> io.setVolts(DEMO_VOLTS.get()))
            .withName("set demo volts(pivot)");
    }

    @Override
    public void periodic() {
        io.refreshData(inputs);
        Logger.processInputs("Wrist", inputs);
    }
}
