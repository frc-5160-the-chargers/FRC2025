package frc.chargers.choreo

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.TimedRobot
import frc.robot.constants.PathfindingPoses
import frc.robot.subsystems.swerve.SwerveConfigurator
import java.io.File
import kotlin.system.exitProcess

/**
 * A dummy robot class that allows you to add choreo variables from code.
 */
class ChoreoVariableWriter : TimedRobot() {
    init {
        val pathfindingPoses = PathfindingPoses(
            Translation2d(Inches.of(-7.5), Inches.of(-15.0)),  // reef offset
            Translation2d(Meters.of(-0.13), Meters.of(-0.5)),  // north source offset,
            Translation2d(Meters.of(-0.13), Meters.of(0.5)),  // south source offset
            SwerveConfigurator.HARDWARE_SPECS
        )
        if (isReal()) throw RuntimeException("Dont run this on the real robot...")
        val choreoFile = ChoreoFile.from(File(Filesystem.getDeployDirectory().path + "/choreo/Autos.chor"))
        choreoFile.setVariable("northSource", pathfindingPoses.eastSourceBlue)
        choreoFile.setVariable("southSource", pathfindingPoses.westSourceBlue)
        choreoFile.write()
        exitProcess(0)
    }
}