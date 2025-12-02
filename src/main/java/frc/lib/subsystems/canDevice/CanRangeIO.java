package frc.lib.subsystems.canDevice;

public interface CanRangeIO {
    void readInputs(CanRangeInputs inputs);

    void updateFrequency(double hz);

}
