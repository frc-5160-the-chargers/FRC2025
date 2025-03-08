package frc.chargers.choreo

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.TimedRobot
import frc.robot.constants.OtherConstants
import frc.robot.constants.TargetPoses
import frc.robot.subsystems.swerve.SwerveConfigurator
import java.io.File
import kotlin.system.exitProcess

/**
 * A dummy robot class that allows you to add choreo variables from code.
 */
class ChoreoVariableWriter : TimedRobot() {
    init {
        val targetPoses = TargetPoses(
            OtherConstants.REEF_SCORE_OFFSET + Translation2d(Inches.of(-1.0), Inches.zero()),
            OtherConstants.SOURCE_OFFSET,
            SwerveConfigurator.HARDWARE_SPECS
        )
        if (isReal()) throw RuntimeException("Dont run this on the real robot...")
        val choreoFile = ChoreoFile.from(File(Filesystem.getDeployDirectory().path + "/choreo/Autos.chor"))
        repeat(12) { i ->
            choreoFile.setVariable("reef$i", targetPoses.reefBlue[i])
        }
        choreoFile.setVariable("northSource", targetPoses.eastSourceBlue)
        choreoFile.setVariable("southSource", targetPoses.westSourceBlue)
        choreoFile.write()
        exitProcess(0)
    }
}