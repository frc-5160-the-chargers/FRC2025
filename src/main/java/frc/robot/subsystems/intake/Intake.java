package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.data.InputStream;
import frc.chargers.data.RobotMode;
import frc.chargers.hardware.MotorDataAutoLogged;
import frc.robot.GlobalState;
import frc.robot.components.LoggedLaserCan;
import frc.robot.components.LoggedLaserCan.LaserCanStatus;
import frc.robot.subsystems.ChargerSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intake extends ChargerSubsystem {
    private static final double
        DISTANCE_TOLERANCE_MM = 0.0,
        OUTTAKE_VOLTAGE = 6.0,
        INTAKE_VOLTAGE = 6.0,
        OUTTAKE_DELAY_SECS = 0.5;

    private final GlobalState globalState;
    private boolean hasCoralInSim = false;

    private final IntakeHardware io = RobotMode.get() == RobotMode.REAL
        ? new SparkIntakeHardware(5, true, 60, false, 5.0)
        : new IntakeHardware();
    @Getter private final MotorDataAutoLogged inputs = new MotorDataAutoLogged();
    private final LoggedLaserCan laserCan = new LoggedLaserCan(5);

    /** A trigger that returns true when the intake detects coral. */
    @AutoLogOutput
    public final Trigger hasCoral = new Trigger(
        () -> (RobotBase.isSimulation() && hasCoralInSim) ||
            (laserCan.inputs.status == LaserCanStatus.SEEN &&
            laserCan.inputs.distance_mm < DISTANCE_TOLERANCE_MM)
    );
    /** A trigger that returns true when the intake is outtaking coral. */
    @AutoLogOutput
    public final Trigger isOuttaking = new Trigger(() -> inputs.appliedVolts > 1.0);

    public Intake(GlobalState globalState) {
        this.globalState = globalState;
    }

    private double getOuttakeVoltage() {
        return globalState.atL1Range ? OUTTAKE_VOLTAGE / 3 : OUTTAKE_VOLTAGE;
    }

    public Command setHasCoralInSimCmd(boolean hasCoral) {
        return Commands.runOnce(() -> hasCoralInSim = hasCoral);
    }

    /**
     * Returns a Command that sets the elevator motors at a specific percent out.
     * A value of 1.0 is equivalent to 12 volts(and vise-versa for -1.0).
     */
    public Command setPowerCmd(InputStream controllerInput) {
        return this.run(() -> io.setVolts(controllerInput.get() * 12))
            .withName("coral set power");
    }

    public Command intakeCmd() {
        return this.run(() -> io.setVolts(INTAKE_VOLTAGE))
            .until(hasCoral)
            .withName("coral intake");
    }

    public Command outtakeCmd() {
        return this.run(() -> io.setVolts(getOuttakeVoltage()))
            .until(hasCoral.negate())
            .andThen(
                this.run(() -> io.setVolts(OUTTAKE_VOLTAGE)).withTimeout(OUTTAKE_DELAY_SECS)
            )
            .withName("coral outtake");
    }

    public Command intakeForeverCmd() {
        return this.run(() -> io.setVolts(INTAKE_VOLTAGE)).withName("coral intake(Forever)");
    }

    public Command outtakeForeverCmd() {
        return this.run(() -> io.setVolts(getOuttakeVoltage())).withName("coral outtake(Forever)");
    }

    public Command idleCmd() {
        return this.run(() -> {
            if (!hasCoral.getAsBoolean()) {
                io.setVolts(0);
                return;
            }
            io.setVolts(globalState.elevatorVelMPS > 0.2 ? -0.4 : -0.25);
        }).withName("Intake Idle");
    }
}