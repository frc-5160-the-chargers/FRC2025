package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.commands.NonBlockingCmds;
import frc.chargers.misc.RobotMode;
import frc.robot.subsystems.ChargerSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
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
    /** Data originating from ElevatorHardware. */
    private final ElevatorDataAutoLogged inputs = new ElevatorDataAutoLogged();

    @AutoLogOutput
    private ExponentialProfile.State currTarget = new ExponentialProfile.State();

    /** Triggers that correspond to various elevator states. */
    @AutoLogOutput
    public final Trigger
        movingUp = new Trigger(() -> inputs.radiansPerSec > 0.1),
        cogLow = new Trigger(() -> heightMeters() < COG_LOW_BOUNDARY.get().in(Meters)),
        hittingLowLimit = new Trigger(
            () -> heightMeters() < MIN_HEIGHT.in(Meters) && inputs.volts < 0
        ),
        hittingHighLimit = new Trigger(
            () -> heightMeters() > MAX_HEIGHT.in(Meters) && inputs.volts > 0
        );

    /**
     * Returns a trigger that is true when the elevator is at the target height,
     * with the default tolerance.
     */
    public Trigger atHeight(Distance height) {
        return atHeight(height, TOLERANCE::get);
    }

    /**
     * Returns a trigger that is true when the elevator is at the target height,
     * with a given tolerance.
     */
    public Trigger atHeight(Distance height, Supplier<Distance> tolerance) {
        return new Trigger(
            () -> Math.abs(heightMeters() - height.in(Meters)) < tolerance.get().in(Meters)
        );
    }

    @AutoLogOutput
    public double heightMeters() {
        return inputs.radians * RADIUS.in(Meters);
    }

    @AutoLogOutput
    public double velocityMPS() {
        return inputs.radiansPerSec * RADIUS.in(Meters);
    }

    public Command idleCmd() {
        return this.run(() -> io.setVolts(0)).withName("Elevator Idle");
    }

    public Command setPowerCmd(DoubleSupplier controllerInput) {
        return this.run(() -> io.setVolts(controllerInput.getAsDouble() * 12))
                .withName("set power (elevator)");
    }

    public Command moveToDemoHeightCmd() {
        // deferred command re-computes the command at runtime
        return Commands.defer(() -> {
            var demoHeight = DEMO_HEIGHT.get();
            if (demoHeight.magnitude() < 0) return Commands.print("Height < 0; demo request ignored.");
            return moveToHeightCmd(demoHeight);
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
            .until(() -> inputs.supplyAmps[0] > 20)
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
        Logger.processInputs(getName(), inputs);

        if (hittingHighLimit.getAsBoolean() || hittingLowLimit.getAsBoolean()) {
            io.setVolts(0);
        }
    }
}