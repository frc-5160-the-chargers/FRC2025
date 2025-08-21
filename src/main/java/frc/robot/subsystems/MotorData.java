//package frc.robot.subsystems;
//
//import com.ctre.phoenix6.hardware.TalonFX;
//
//public class MotorData {
//    public double positionRad = 0;
//    public double velocityRadPerSec = 0;
//    public double[] motorTemps = new double[0];
//    public double[] statorCurrent = new double[0];
//    public double[] supplyCurrents = new double[0];
//    public double[] appliedVoltages = new double[0];
//
//    private void updateArraySizes(int length) {
//        if (motorTemps.length == length) return;
//        motorTemps = new double[length];
//        statorCurrent = new double[length];
//        supplyCurrents = new double[length];
//        appliedVoltages = new double[length];
//    }
//
//    public void update(TalonFX leader, TalonFX... followers) {
//        updateArraySizes(followers.length + 1);
//        var positionSig = leader.getPosition(false);
//        var velocitySig = leader.getVelocity(false);
//
//        positionRad =
//    }
//}
