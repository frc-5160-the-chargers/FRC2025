package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chargers.hardware.SignalBatchRefresher;
import frc.robot.components.controllers.DriverController;
import frc.robot.subsystems.drive.SwerveDrive;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
    private final SwerveDrive drive = new SwerveDrive();
    private final DriverController controller = new DriverController();

    public Robot() {
        setUseTiming(true);
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        drive.setDefaultCommand(
            drive.driveCmd(controller.forwardOutput, controller.strafeOutput, controller.rotationOutput, true)
        );
        drive.resetPose(new Pose2d(5, 7, Rotation2d.kZero));
    }

    @Override
    public void robotPeriodic() {
        SignalBatchRefresher.refreshAll();
        CommandScheduler.getInstance().run();
        SimulatedArena.getInstance().simulationPeriodic();
    }
}
