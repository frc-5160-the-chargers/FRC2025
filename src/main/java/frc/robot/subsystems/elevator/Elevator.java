package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.data.InputStream;
import frc.chargers.data.TunableValues.TunableNum;
import frc.chargers.hardware.MotorInputsAutoLogged;
import frc.chargers.utils.RobotMode;
import frc.robot.subsystems.Superstructure.SharedData;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConsts.*;

public class Elevator extends SubsystemBase {
    private final TunableNum demoHeight = new TunableNum("Elevator/DemoHeightMeters", 0.5);
    private final TunableNum demoVolts = new TunableNum("Elevator/DemoVolts", 3);
    private final TunableNum kP = new TunableNum("Elevator/KP", 0.5);
    private final TunableNum kD = new TunableNum("Elevator/KD", 0);

    private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            MAX_LINEAR_VEL.in(MetersPerSecond),
            MAX_LINEAR_ACCEL.in(MetersPerSecondPerSecond)
        )
    );
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);
    @AutoLogOutput private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
    private final SharedData sharedData;
    private final ElevatorHardware io;
    private final MotorInputsAutoLogged inputData = new MotorInputsAutoLogged();

    @AutoLogOutput
    public final Trigger movingUp =
        new Trigger(() -> inputData.velocityRadPerSec > 0.1);

    @AutoLogOutput
    public final Trigger atLowPosition =
        new Trigger(() -> heightMeters() < COG_LOW_BOUNDARY.in(Meters));

    @AutoLogOutput
    public final Trigger hittingLowLimit =
        new Trigger(() -> heightMeters() < MIN_HEIGHT.in(Meters) && inputData.appliedVoltage[0] < 0);

    @AutoLogOutput
    public final Trigger hittingHighLimit =
        new Trigger(() -> heightMeters() > MAX_HEIGHT.in(Meters) && inputData.appliedVoltage[0] > 0);

    public Elevator(SharedData sharedData) {
        this.io = switch (RobotMode.get()) {
            case REAL -> new RealElevatorHardware();
            case SIM -> new SimElevatorHardware();
            case REPLAY -> new ElevatorHardware();
        };
        this.sharedData = sharedData;
        io.setPDGains(kP.get(), kD.get());
        kP.onChange(p -> io.setPDGains(p, kD.get()));
        kD.onChange(d -> io.setPDGains(kP.get(), d));
        setDefaultCommand(idleCmd());
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
        return inputData.positionRad * RADIUS.in(Meters);
    }

    @AutoLogOutput
    public double velocityMPS() {
        return inputData.velocityRadPerSec * RADIUS.in(Meters);
    }

    @AutoLogOutput
    public double gravityCompensationV() {
        if (RobotBase.isSimulation()) return 0;
        return sharedData.hasCoral ? WITH_CORAL_KG : NO_CORAL_KG;
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
            var demoHeight = this.demoHeight.get();
            if (demoHeight < 0) return Commands.print("Height < 0; demo request ignored.");
            return moveToHeightCmd(Meters.of(demoHeight));
        }, Set.of(this));
    }

    public Command moveToHeightCmd(Distance target) {
        var goal = new TrapezoidProfile.State(target.in(Meters), 0);
        return Commands.runOnce(() -> {
            currentSetpoint = new TrapezoidProfile.State(heightMeters(), velocityMPS());
            Logger.recordOutput("Elevator/targetHeight", target.in(Meters));
        }).andThen(
            this.run(() -> {
                currentSetpoint = trapezoidProfile.calculate(0.02, currentSetpoint, goal);
                double ffOutput = feedforward.calculate(currentSetpoint.velocity);
                ffOutput += gravityCompensationV();
                Logger.recordOutput("Elevator/feedforward", ffOutput);
                io.setRadians(currentSetpoint.position, ffOutput);
            }).until(atHeight(target))
        ).finallyDo(() -> io.setVolts(0));
    }

    public Command currentZeroCmd() {
        return this.run(() -> io.setVolts(-0.5))
            .until(() -> inputData.supplyCurrent[0] > 20)
            .finallyDo((interrupted) -> {
                if (!interrupted) {
                    io.zeroEncoder();
                    currentSetpoint = new TrapezoidProfile.State();
                }
            })
            .withName("current zero command(Elevator)");
    }

    public Command setDemoVoltageCmd() {
        return this.run(() -> io.setVolts(demoVolts.get()));
    }

    @Override
    public void periodic() {
        io.refreshData(inputData);
        // If real/sim mode, log the data.
        // If replay mode, overrides the data with data from the log file instead.
        Logger.processInputs("Elevator", inputData);

        if (hittingHighLimit.getAsBoolean() || hittingLowLimit.getAsBoolean()) {
            io.setVolts(0);
        }
        sharedData.elevatorVelMPS = inputData.velocityRadPerSec * RADIUS.in(Meters);
        sharedData.atL1Range = heightMeters() < 0.2;
    }
}
