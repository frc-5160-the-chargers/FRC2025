package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.commands.NonBlockingCmds;
import frc.chargers.data.InputStream;
import frc.chargers.data.RobotMode;
import frc.robot.subsystems.ChargerSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConsts.*;

public class Elevator extends ChargerSubsystem {
    private final ExponentialProfile profile = new ExponentialProfile(
        ExponentialProfile.Constraints.fromStateSpace(
            9,
            ELEVATOR_SYSTEM.getA(1, 1),
            ELEVATOR_SYSTEM.getB(1, 0)
        )
    );
    private final ElevatorFeedforward feedforwardEq = new ElevatorFeedforward(KS, 0.0, KV);
    private final ElevatorHardware io = switch (RobotMode.get()) {
        case SIM -> new SimElevatorHardware();
        case REAL, REPLAY -> new ElevatorHardware();
    };
    private final ElevatorDataAutoLogged inputs = new ElevatorDataAutoLogged();

    @AutoLogOutput
    private ExponentialProfile.State currTarget = new ExponentialProfile.State();

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

    public Elevator() {
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
        return new Trigger(
            () -> Math.abs(heightMeters() - height.in(Meters)) < tolerance.in(Meters)
        );
    }

    @AutoLogOutput
    public double heightMeters() {
        return inputs.positionRad * RADIUS.in(Meters);
    }

    @AutoLogOutput
    public double velocityMPS() {
        return inputs.velocityRadPerSec * RADIUS.in(Meters);
    }

    public Command idleCmd() {
        return this.run(() -> io.setVolts(0)).withName("Elevator Idle");
    }

    public Command setPowerCmd(InputStream controllerInput) {
        return this.run(() -> io.setVolts(controllerInput.get() * 12))
                .withName("set power (elevator)");
    }

    public Command moveToDemoHeightCmd() {
        // deferred command re-computes the command at runtime
        return Commands.defer(() -> {
            var demoHeight = DEMO_HEIGHT.get();
            if (demoHeight < 0) return Commands.print("Height < 0; demo request ignored.");
            return moveToHeightCmd(Meters.of(demoHeight));
        }, Set.of(this));
    }

    public Command moveToHeightCmd(Distance target) {
        var finalTarget = new ExponentialProfile.State(target.in(Meters), 0);
        return NonBlockingCmds.sequence(
            this.runOnce(() -> {
                currTarget = new ExponentialProfile.State(heightMeters(), velocityMPS());
                Logger.recordOutput(key("FinalGoalMeters"), target);
            }),
            this.run(() -> {
                var nextTarget = profile.calculate(0.02, currTarget, finalTarget);
                double ffOutput = feedforwardEq.calculateWithVelocities(currTarget.velocity, nextTarget.velocity);
                Logger.recordOutput(key("Feedforward Volts"), ffOutput);
                io.setRadians(currTarget.position / RADIUS.in(Meters), ffOutput);
                currTarget = nextTarget;
            })
        )
            .until(atHeight(target))
            .withName("MoveToHeight (" + target.in(Meters) + " m)");
    }

    public Command currentZeroCmd() {
        return this.run(() -> io.setVolts(-0.5))
            .until(() -> inputs.supplyCurrent[0] > 20)
            .finallyDo((interrupted) -> {
                if (!interrupted) {
                    io.zeroEncoder();
                    currTarget = new ExponentialProfile.State();
                }
            })
            .withName("Elevator Current Zero");
    }

    public Command setDemoVoltageCmd() {
        return this.run(() -> io.setVolts(DEMO_VOLTS.get()));
    }

    @Override
    public void loggedPeriodic() {
        io.refreshData(inputs);
        // If real/sim mode, log the data.
        // If replay mode, overrides the data with data from the log file instead.
        Logger.processInputs("Elevator", inputs);

        if (hittingHighLimit.getAsBoolean() || hittingLowLimit.getAsBoolean()) {
            io.setVolts(0);
        }
    }
}