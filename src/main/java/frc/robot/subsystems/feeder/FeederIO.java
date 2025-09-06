package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  class FeederIOInputs {
    public RollerData rightRollerData = new RollerData(false, 0, 0, 0, 0, 0);
    public RollerData leftRollerData = new RollerData(false, 0, 0, 0, 0, 0);
    public CanRangeData canRangeData = new CanRangeData(false, false, 0, 0);
  }

  /** roller-related telemetry. */
  record RollerData(
      boolean isMotorConnected,
      double voltage,
      double supplyCurrent,
      double statorCurrent,
      double temperature,
      double velocityRPS) {}

  record CanRangeData(
      boolean isSensorConnected, boolean tripped, double signalStrength, double distanceMeters) {}

  default void updateInputs(FeederIOInputs inputs) {}

  default void setRollerVoltage(double voltage) {}

  default void checkForJam() {}
}
