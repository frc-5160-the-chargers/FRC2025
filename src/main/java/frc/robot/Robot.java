package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.chargers.utils.ExtensionMethods;
import frc.chargers.utils.Logger;
import lombok.experimental.ExtensionMethod;

@Logged
@ExtensionMethod({ExtensionMethods.class})
public class Robot extends TimedRobot {
    public Robot() {
        Epilogue.bind(this);
        Logger.startDefault();
    }
}
