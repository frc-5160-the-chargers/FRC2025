package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
    public Robot() {
        setUseTiming(true);
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
