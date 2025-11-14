//package frc.robot.subsystems;
//
//import com.revrobotics.spark.SparkFlex;
//import com.revrobotics.spark.SparkLowLevel;
//import com.revrobotics.spark.config.SparkFlexConfig;
//import edu.wpi.first.wpilibj.simulation.DCMotorSim;
//import edu.wpi.first.wpilibj2.command.Command;
//
//public class Intake extends ChargerSubsystem {
//    private final SparkFlex motor = new SparkFlex(0, SparkLowLevel.MotorType.kBrushless);
//    private final DCMotorSim motorSim = new DCMotorSim()
//
//    public Intake() {
//        setDefaultCommand(
//            this.run(() -> {
//                motor.setVoltage(0.5);
//            })
//        );
//        // 1. We DONT KNOW what hardware/motors we're gonna use
//        // 2.
//    }
//
//    private void intake(){
//        motor.setVoltage(6);
//    }
//    private void outtake(){
//        motor.setVoltage(-6);
//    }
//    private void stop(){
//        motor.setVoltage(0);
//    }
//
//    public Command outtakeForSeconds(double duration) {
//        // idiomatic
//        return this.run(this::outtake)
//            .withTimeout(duration)
//            .finallyDo(this::stop)
//            .withName("Outtake");
//    }
//
//    public Command intakeForSeconds(double duration) {
//        return this.run(this::intake)
//            .withTimeout(duration)
//            .finallyDo(this::stop)
//            .withName("Intake");
//    }
//
//    public Command intakeOutake() {
//        return intakeForSeconds(3.0)
//            .andThen(outtakeForSeconds(3.0));
//    }
//}
