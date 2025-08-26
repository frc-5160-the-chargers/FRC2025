package frc.chargers.hardware;

public interface MotorIO<D extends MotorData> {
    void refreshData(D data);

    void setVolts(double volts);

    void setTorqueCurrent(double amps);

    void setCoastMode(boolean enabled);
}
