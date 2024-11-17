package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.chargers.utils.ChargerExtensions;
import frc.chargers.utils.Logger;
import lombok.experimental.ExtensionMethod;

@Logged
@ExtensionMethod({ChargerExtensions.class})
public class Robot extends TimedRobot {
    public Robot() {
        Epilogue.bind(this);
        Logger.configureDefault();
    }
}
