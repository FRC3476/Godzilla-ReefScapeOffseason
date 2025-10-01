package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  class FeederIOInputs {
    public F_RollerData rightRollerData = new F_RollerData(false, 0, 0, 0, 0, 0, false);
    public F_RollerData leftRollerData = new F_RollerData(false, 0, 0, 0, 0, 0, false);
    public F_CanRangeData canRangeData = new F_CanRangeData(false, false, 0, 0);
  }

  /** roller-related telemetry. */
  record F_RollerData(
      boolean isMotorConnected,
      double voltage,
      double supplyCurrent,
      double statorCurrent,
      double temperature,
      double velocityRPS,
      boolean isMotorStalled) {}

  record F_CanRangeData(
      boolean isSensorConnected, boolean tripped, double signalStrength, double distanceMeters) {}

  default void updateInputs(FeederIOInputs inputs) {}

  default void setRollerVoltage(double voltage) {}

  default void setRollerVoltageReversed(double voltage) {}

  default boolean checkMotorsStalled() {
    return false;
  }

  default double whichMotorStalled() {
    return 0;
  }
}
