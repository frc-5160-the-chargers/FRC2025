package frc.chargers.choreo

import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.TimedRobot
import frc.chargers.field.ScoringPoses
import java.io.File
import kotlin.system.exitProcess

/**
 * A dummy robot class that allows you to add choreo variables from code.
 */
class ChoreoVariableWriter : TimedRobot() {
    init {
        if (isReal()) throw RuntimeException("Dont run this on the real robot...")
        val choreoFile = ChoreoFile.from(File(Filesystem.getDeployDirectory().path + "/choreo/Autos.chor"))
        repeat(12) { idx ->
            choreoFile.setVariable("reef$idx", ScoringPoses.getPose(idx).orElseThrow())
        }
        choreoFile.write()
        exitProcess(0)
    }
}
