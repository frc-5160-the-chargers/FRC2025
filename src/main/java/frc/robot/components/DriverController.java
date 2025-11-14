package frc.robot.components;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.chargers.data.InputStream;
import org.littletonrobotics.junction.AutoLogOutput;

import static edu.wpi.first.wpilibj.Alert.AlertType.kWarning;
import static frc.chargers.commands.TriggerUtil.bind;

public class DriverController extends CommandPS5Controller { // does not have a rumble command due to problems
    public DriverController() {
        super(0);
        bind(new Alert("Driver Disconnected", kWarning), () -> !super.isConnected());
    }

    @AutoLogOutput
    private final InputStream slowModeOutput =
        InputStream.of(this::getL2Axis)
            .deadband(0.2, 1)
            .map(it -> 1 - it / 2);

    @AutoLogOutput
    public final InputStream
        forwardOutput = InputStream.of(this::getLeftY)
            .deadband(0.2, 1)
            .times(slowModeOutput)
            .negate(),
        strafeOutput = InputStream.of(this::getLeftX)
            .deadband(0.2, 1)
            .times(slowModeOutput)
            .negate(),
        rotationOutput = InputStream.of(this::getRightX)
            .deadband(0.2, 1)
            .times(slowModeOutput)
            .negate();
}