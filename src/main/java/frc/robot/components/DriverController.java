package frc.robot.components;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import org.littletonrobotics.junction.AutoLogOutput;

import static edu.wpi.first.wpilibj.Alert.AlertType.kWarning;
import static frc.chargers.commands.TriggerUtil.bind;

public class DriverController extends CommandPS5Controller {
    public DriverController() {
        super(0);
        bind(new Alert("Driver Disconnected", kWarning), () -> !super.isConnected());
    }

    @AutoLogOutput
    private double slowModeOutput() {
        double output = getL2Axis();
        output = MathUtil.applyDeadband(output, 0.2, 1);
        output = (2 - output);
        return output;
    }

    private double modifyDriveAxis(double output) {
        output = MathUtil.applyDeadband(output, 0.2, 1);
        output *= slowModeOutput();
        return output * -1;
    }

    @AutoLogOutput
    public double forwardOutput() {
        return modifyDriveAxis(getLeftY());
    }

    @AutoLogOutput
    public double strafeOutput() {
        return modifyDriveAxis(getLeftX());
    }

    @AutoLogOutput
    public double rotationOutput() {
        return modifyDriveAxis(getRightX());
    }
}