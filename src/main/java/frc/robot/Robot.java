package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chargers.utils.UtilExtensionMethods;
import frc.chargers.utils.Logger;
import frc.chargers.utils.ChassisPowers;
import frc.chargers.utils.swerve.SwerveDriveBase;
import frc.robot.subsystems.SwerveDrive;
import lombok.experimental.ExtensionMethod;
import org.ironmaple.simulation.SimulatedArena;

import static edu.wpi.first.epilogue.Logged.Strategy.OPT_IN;

@Logged(strategy = OPT_IN)
@ExtensionMethod({UtilExtensionMethods.class})
public class Robot extends TimedRobot {
    @Logged private final SwerveDriveBase drivetrain;
    private final CommandXboxController driverController;
    
    public Robot() {
        Epilogue.bind(this);
        addPeriodic(CommandScheduler.getInstance()::run, 0.02);
        if (isSimulation()) {
            addPeriodic(SimulatedArena.getInstance()::simulationPeriodic, 0.02);
            Epilogue.getConfig().errorHandler = ErrorHandler.crashOnError();
        }
        Logger.configureDefault();
        
        drivetrain = new SwerveDrive("drivetrain");
        driverController = new CommandXboxController(0);
        
        drivetrain.resetPose(new Pose2d(5.0, 7.0, Rotation2d.kZero));
        drivetrain.setDefaultCommand(
            drivetrain.teleopDriveCmd(
                () -> new ChassisPowers(
                    -driverController.getLeftY(),
                    -driverController.getLeftX(),
                    driverController.getRightX()
                ),
                true
            )
        );
    }
}
