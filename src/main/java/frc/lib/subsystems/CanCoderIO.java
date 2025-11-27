package frc.lib.subsystems;

public interface CanCoderIO {
  void readInputs(CanCoderInputs inputs);

  void updateFrequency(double hz);

  void setMagnetOffset(double offset);

  double getAbsolutePositionSlow();

  void setPosition(double position);
}
