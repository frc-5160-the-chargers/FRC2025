package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.robot.subsystems.swerve.SwerveDrive;

@Logged
public class KitBotCoralOut extends StandardSubsystem {
    private final Motor motor;
    private boolean hasCoralInSim = false;
    private SwerveDrive drive;

    public KitBotCoralOut(SwerveDrive drive) {
//        if (RobotBase.isSimulation()) {
        this.drive = drive;

        motor = new SimMotor(
                SimMotor.SimMotorType.DC(DCMotor.getNEO(1), 0.004),
                null
        );
//        }
    }

    public Command simulateHasCoral(SwerveDrive drive) {
        return Commands.runOnce(() -> hasCoralInSim = true);
    }

    public Command outtakeCmd() {
        return Commands.runOnce(() -> hasCoralInSim = false).andThen(this.run(() -> motor.setVoltage(12)).withTimeout(1.0));
//        return this.run(() -> motor.setVoltage(12)).withTimeout(1.0);

    }

    @Override
    public Command stopCmd() {
        return this.runOnce(() -> motor.setVoltage(0));
    }

    @Override
    public void periodic() {
        if (hasCoralInSim) {
            log("Coral Pos",
                new Pose3d(
                    drive.getPose().getX(),
                    drive.getPose().getY(),
                    0.5, new Rotation3d(0,0, Math.PI/2))
            );
        } else {
            log("Coral Pos", new Pose3d(-5,-5,-5, Rotation3d.kZero));
        }
    }
}
