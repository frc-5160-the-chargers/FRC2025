package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chargers.hardware.SignalBatchRefresher;
import frc.chargers.misc.Tracer;
import frc.robot.components.DriverController;
import frc.robot.subsystems.drive.SwerveDrive;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

@SuppressWarnings("ALL")
public class DriverTrainingEnv extends LoggedRobot {
    private final DriverController
        controller1 = new DriverController(0),
        controller2 = new DriverController(1);

    private final SwerveDrive
        drivetrain1 = new SwerveDrive("SwerveDrive1"),
        drivetrain2 = new SwerveDrive("SwerveDrive2");

    public DriverTrainingEnv() {
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();

        drivetrain1.setDefaultCommand(drivetrain1.driveCmd(controller1::getSwerveRequest));
        drivetrain2.setDefaultCommand(drivetrain2.driveCmd(controller2::getSwerveRequest));

        new Notifier(SimulatedArena.getInstance()::simulationPeriodic)
            .startPeriodic(0.002);
    }

    @Override
    public void robotPeriodic() {
        Tracer.trace("Command Scheduler", CommandScheduler.getInstance()::run);
        SignalBatchRefresher.refreshAll();
    }
}
