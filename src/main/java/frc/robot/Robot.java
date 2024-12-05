package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chargers.utils.UtilExtensionMethods;
import frc.chargers.utils.Logger;
import lombok.experimental.ExtensionMethod;
import org.ironmaple.simulation.SimulatedArena;

import static edu.wpi.first.epilogue.Logged.Strategy.OPT_IN;

@Logged(strategy = OPT_IN)
@ExtensionMethod({UtilExtensionMethods.class})
public class Robot extends TimedRobot {
    
    @Logged
    public boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
    }
    
    public Robot() {
        Epilogue.bind(this);
        if (isSimulation()) Epilogue.getConfig().errorHandler = ErrorHandler.crashOnError();
        Logger.configureDefault();
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (isSimulation()) SimulatedArena.getInstance().simulationPeriodic();
    }
}
