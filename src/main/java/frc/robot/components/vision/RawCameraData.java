package frc.robot.components.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;

public class RawCameraData implements LoggableInputs {
    public boolean connected = true;
    public List<PhotonPipelineResult> results = new ArrayList<>();

    /** Pushes data to a log file. */
    @Override
    public void toLog(LogTable logTable) {
        logTable.put("connected", connected);
        logTable.put("numResults", results.size());
        for (int i = 0; i < results.size(); i++) {
            logTable.put("results/" + i, results.get(i));
        }
    }

    /** Overrides the results with data from another file, effectively "replaying" a previous log. */
    @Override
    public void fromLog(LogTable logTable) {
        results.clear();
        connected = logTable.get("connected", false);
        int numResults = logTable.get("numResults", 0);
        for (int i = 0; i < numResults; i++) {
            results.add(logTable.get("results/" + i, new PhotonPipelineResult()));
        }
    }
}
