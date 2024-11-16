package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.chargers.utils.ExtensionMethods;
import lombok.experimental.ExtensionMethod;

@Logged
@ExtensionMethod({ExtensionMethods.class})
public class Robot extends TimedRobot {
    public Robot() {
        Epilogue.bind(this);
        DogLog.setOptions(DogLog.getOptions().withNtPublish(true).withLogExtras(true));
        DogLog.setPdh(new PowerDistribution());
        
        
    }
}
