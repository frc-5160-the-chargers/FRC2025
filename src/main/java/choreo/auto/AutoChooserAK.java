package choreo.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class AutoChooserAK extends AutoChooserImpl {
    private String selected = NONE_NAME;
    private final LoggableInputs replayHandle = new LoggableInputs() {
        @Override
        public void toLog(LogTable table) {
            table.put("selected", selected);
        }

        @Override
        public void fromLog(LogTable table) {
            selected = table.get("selected", NONE_NAME);
        }
    };

    public AutoChooserAK(String name) {
        setOnSelectOverride(value -> {
            selected = value;
            Logger.processInputs(name, replayHandle);
            return selected;
        });
        SmartDashboard.putData(name, this);
    }
}
