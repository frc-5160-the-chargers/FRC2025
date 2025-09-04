package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.utils.RobotMode;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeHardware;
import frc.robot.subsystems.intake.SimIntakeHardware;
import frc.robot.subsystems.intake.SparkIntakeHardware;
import frc.robot.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure {
    public static class SharedData {
        @AutoLogOutput public boolean atL1Range = false;
        public Rotation2d heading = Rotation2d.kZero;
        public double elevatorVelMPS = 0;
        public boolean hasCoral = false;
    }

    public enum State {
        STOW,
        WRIST_OUT,
        WRIST_INWARD,
        INTAKE,
        L1, L2, L3, L4,
    }

    @AutoLogOutput private State goalState = State.STOW;
    @AutoLogOutput private State currentState = State.STOW;
    private final SharedData sharedData = new SharedData();

    public final Elevator elevator = new Elevator(sharedData);
    public final Wrist wrist = new Wrist(sharedData);
    public final Intake intake = new Intake(
        "Intake",
        switch (RobotMode.get()) {
            case REAL -> new SparkIntakeHardware(5, true, 60, false, 5.0);
            case SIM -> new SimIntakeHardware(DCMotor.getNeoVortex(1), 0.025, 3.0);
            case REPLAY -> new IntakeHardware();
        }
    );

    private Trigger areIdle(Subsystem... subsystems) {
        return new Trigger(() -> {
            for (var s: subsystems) {
                if (s.getCurrentCommand() != s.getDefaultCommand()) return false;
            }
            return true;
        });
    }

    private Trigger goalIs(State goal) {
        return new Trigger(() -> this.goalState == goal);
    }

    private Trigger goalIsOneOf(State... goals) {
        return new Trigger(() -> {
            for (var goal: goals) {
                if (this.goalState == goal) return true;
            }
            return false;
        });
    }

    private Trigger at(State target) {
        return new Trigger(() -> this.currentState == target);
    }

    private Trigger atOneOf(State... targets) {
        return new Trigger(() -> {
            for (var target: targets) {
                if (this.currentState == target) return true;
            }
            return false;
        });
    }

    public Superstructure() {
        goalIsOneOf(State.L1, State.L2, State.L3, State.L4)
            .and(at(State.WRIST_OUT).negate());
    }

}
