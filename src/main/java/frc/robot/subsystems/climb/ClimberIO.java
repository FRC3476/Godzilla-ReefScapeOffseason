package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public ClimberIOData data = new ClimberIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  record ClimberIOData(
      boolean motorConnected,
      double positionRads,
      double velocityRadsPerSec,
      double appliedVoltage,
      double supplyCurrentAmps,
      double statorCurrentAmps,
      double tempCelsius) {}

  default void runVolts(double volts) {}

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setZero() {}

  default boolean checkClimbMotorStalled() {
    return false;
  }

  default void updateClimbReady() {}

  default boolean isClimbReady() {
    return false;
  }
}
