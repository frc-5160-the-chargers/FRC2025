package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.data.InputStream;
import frc.chargers.hardware.MotorDataAutoLogged;
import frc.chargers.data.RobotMode;
import frc.robot.subsystems.GlobalState;
import frc.robot.subsystems.ChargerSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConsts.*;

public class Elevator extends ChargerSubsystem {
    private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            MAX_LINEAR_VEL.in(MetersPerSecond),
            MAX_LINEAR_ACCEL.in(MetersPerSecondPerSecond)
        )
    );
    private final ElevatorHardware io = switch (RobotMode.get()) {
        case REAL -> new RealElevatorHardware();
        case SIM -> new SimElevatorHardware();
        case REPLAY -> new ElevatorHardware();
    };
    private final MotorDataAutoLogged inputs = new MotorDataAutoLogged();
    private final GlobalState globalState;

    private TrapezoidProfile.State currentS = new TrapezoidProfile.State();

    /** Triggers that correspond to various elevator states. */
    @AutoLogOutput
    public final Trigger
        movingUp = new Trigger(() -> inputs.velocityRadPerSec > 0.1),
        cogLow = new Trigger(() -> heightMeters() < COG_LOW_BOUNDARY.in(Meters)),
        hittingLowLimit = new Trigger(
            () -> heightMeters() < MIN_HEIGHT.in(Meters) && inputs.appliedVolts < 0
        ),
        hittingHighLimit = new Trigger(
            () -> heightMeters() > MAX_HEIGHT.in(Meters) && inputs.appliedVolts > 0
        );

    public Elevator(GlobalState globalState) {
        this.globalState = globalState;
        io.setPDGains(KP.get(), KD.get());
        KP.onChange(p -> io.setPDGains(p, KD.get()));
        KD.onChange(d -> io.setPDGains(KP.get(), d));
    }

    /**
     * Returns a trigger that is true when the elevator is at the target height,
     * with the default tolerance.
     */
    public Trigger atHeight(Distance height) {
        return atHeight(height, TOLERANCE);
    }

    /**
     * Returns a trigger that is true when the elevator is at the target height,
     * with a given tolerance.
     */
    public Trigger atHeight(Distance height, Distance tolerance) {
        return new Trigger(() -> Math.abs(heightMeters() - height.in(Meters)) < TOLERANCE.in(Meters));
    }

    @AutoLogOutput
    public double heightMeters() {
        return inputs.positionRad * RADIUS.in(Meters);
    }

    @AutoLogOutput
    public double velocityMPS() {
        return inputs.velocityRadPerSec * RADIUS.in(Meters);
    }

    @AutoLogOutput
    public double gravityCompensationV() {
        if (RobotBase.isSimulation()) return 0;
        return globalState.hasCoral ? WITH_CORAL_KG : NO_CORAL_KG;
    }

    public Command idleCmd() {
        return this.run(() -> io.setVolts(0)).withName("Elevator Idle");
    }

    public Command setPowerCmd(InputStream controllerInput) {
        return this.run(() -> io.setVolts(
            controllerInput.get() * 12 + gravityCompensationV() + KS
        )).withName("set power (elevator)");
    }

    public Command moveToDemoHeightCmd() {
        // deferred command re-computes the command at runtime; dw abt this if u dont understand
        return Commands.defer(() -> {
            var demoHeight = DEMO_HEIGHT.get();
            if (demoHeight < 0) return Commands.print("Height < 0; demo request ignored.");
            return moveToHeightCmd(Meters.of(demoHeight));
        }, Set.of(this));
    }

    public Command moveToHeightCmd(Distance target) {
        var goal = new TrapezoidProfile.State(target.in(Meters), 0);
        return Commands.runOnce(() -> {
            currentS = new TrapezoidProfile.State(heightMeters(), velocityMPS());
            Logger.recordOutput(key("targetHeight"), target);
        }).andThen(
            this.run(() -> {
                currentS = trapezoidProfile.calculate(0.02, currentS, goal);
                double ffOutput = Math.signum(currentS.velocity) * KS
                    + currentS.velocity * KV
                    + gravityCompensationV();
                Logger.recordOutput(key("feedforward"), ffOutput);
                io.setRadians(currentS.position, ffOutput);
            }).until(atHeight(target))
        ).finallyDo(() -> io.setVolts(0));
    }

    public Command currentZeroCmd() {
        return this.run(() -> io.setVolts(-0.5))
            .until(() -> inputs.supplyCurrent[0] > 20)
            .finallyDo((interrupted) -> {
                if (!interrupted) {
                    io.zeroEncoder();
                    currentS = new TrapezoidProfile.State();
                }
            })
            .withName("current zero command(Elevator)");
    }

    public Command setDemoVoltageCmd() {
        return this.run(() -> io.setVolts(DEMO_VOLTS.get()));
    }

    @Override
    public void periodic() {
        io.refreshData(inputs);
        // If real/sim mode, log the data.
        // If replay mode, overrides the data with data from the log file instead.
        Logger.processInputs("Elevator", inputs);

        if (hittingHighLimit.getAsBoolean() || hittingLowLimit.getAsBoolean()) {
            io.setVolts(0);
        }
        globalState.elevatorVelMPS = inputs.velocityRadPerSec * RADIUS.in(Meters);
        globalState.atL1Range = heightMeters() < 0.2;
    }
}
