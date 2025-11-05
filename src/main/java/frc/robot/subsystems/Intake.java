package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends ChargerSubsystem{
    private final SparkFlex motor = new SparkFlex(0, SparkLowLevel.MotorType.kBrushless);

    private void intake(){
        motor.setVoltage(6);
    }
    private void outtake(){
        motor.setVoltage(-6);
    }
    private void stop(){
        motor.setVoltage(0);
    }

    public Command outtakeFor3Seconds() {
        return this.run(() -> {
            intake();
        });
    }

    public Command intakeFor3Seconds() {
        return this.run(() -> {});
    }
}
