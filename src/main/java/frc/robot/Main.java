package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.chargers.choreo.ChoreoVariableWriter;

/**
 * Warning: Do not edit this class
 * aside from replacing "Robot::new"
 * with "CustomRobotName::new".
 */
public class Main {
    public static void main(String[] args) {
        RobotBase.startRobot(ChoreoVariableWriter::new);
    }
}
