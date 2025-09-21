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
      double torqueCurrentAmps,
      double supplyVolts,
      double tempCelsius) {}

  default void runVolts(double volts) {}

  default void setClimbPosition(double position) {}

  default void updateInputs(ClimberIOInputs inputs) {}
}
