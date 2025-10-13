package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.utils.RobotMode;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeHardware;
import frc.robot.subsystems.intake.SimIntakeHardware;
import frc.robot.subsystems.intake.SparkIntakeHardware;
import frc.robot.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.AutoLogOutput;

import static edu.wpi.first.units.Units.*;


public class Superstructure extends SubsystemBase {
    public static class SharedData {
        @AutoLogOutput public boolean atL1Range = false;
        public Rotation2d heading = Rotation2d.kZero;
        public double elevatorVelMPS = 0;
        public boolean hasCoral = false;
    }

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


}
