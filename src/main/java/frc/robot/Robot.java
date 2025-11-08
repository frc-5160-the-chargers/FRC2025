package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chargers.data.RobotMode;
import frc.chargers.hardware.SignalBatchRefresher;
import frc.robot.components.controllers.DriverController;
import frc.robot.subsystems.drive.SwerveDrive;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;

public class Robot extends LoggedRobot {
    private final SwerveDrive drive = new SwerveDrive();
    private final DriverController controller = new DriverController();
    private final Pigeon2 dummyPigeon = new Pigeon2(0);

    public Robot() {
        setUseTiming(true);
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.start();
        drive.setDefaultCommand(
            drive.driveCmd(controller.forwardOutput, controller.strafeOutput, controller.rotationOutput, false)
        );
        drive.resetPose(new Pose2d(5, 7, Rotation2d.kZero));
        controller.triangle().whileTrue(drive.rickRollCmd());
        autonomous().whileTrue(
            drive.runDriveMotors()
        );
    }

    @Override
    public void robotPeriodic() {
        SignalBatchRefresher.refreshAll();
        CommandScheduler.getInstance().run();
        if (RobotMode.isSim()) {
            SimulatedArena.getInstance().simulationPeriodic();
        }
    }
}
