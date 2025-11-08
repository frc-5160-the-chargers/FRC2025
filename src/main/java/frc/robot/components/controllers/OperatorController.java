package frc.robot.components.controllers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chargers.data.InputStream;
import org.littletonrobotics.junction.AutoLogOutput;

import static edu.wpi.first.wpilibj.Alert.AlertType.kWarning;
import static frc.chargers.commands.TriggerUtil.bind;

public class OperatorController extends CommandXboxController implements Subsystem {
    public OperatorController() {
        super(1);
        this.register();
        bind(new Alert("Operator Disconnected", kWarning), () -> !super.isConnected());
    }

    public Command rumbleCmd(RumbleType type, double time) {
        return this.runOnce(() -> setRumble(type, 0.5))
            .andThen(
                Commands.waitSeconds(time),
                this.runOnce(() -> setRumble(type, 0.5))
            )
            .withName("operator controller rumble");
    }

    @AutoLogOutput
    public final InputStream
        manualElevatorInput = InputStream.of(this::getLeftY)
            .deadband(0.1, 1)
            .times(-0.25),
        manualPivotInput = InputStream.of(this::getRightY)
            .deadband(0.1, 1)
            .times(-0.15),
        climbUpInput = InputStream.of(this::getRightTriggerAxis)
            .deadband(0.1, 1)
            .times(0.2),
        climbDownInput = InputStream.of(this::getLeftTriggerAxis)
            .deadband(0.1, 1)
            .times(0.6)
            .negate();
}
