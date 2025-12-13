package frc.robot.subsystems.end_effector;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {

  @AutoLog
  class ClawIOInputs {
    public EE_RollerData rollerData = new EE_RollerData(false, 0, 0, 0, 0, 0);
    public EE_CANRangeData firstCANRangeData = new EE_CANRangeData(false, false);
    public EE_CANRangeData secondCANRangeData = new EE_CANRangeData(false, false);
    public EE_CANRangeData scoringCANRangeData = new EE_CANRangeData(false, false);
  }

  record EE_RollerData(
      boolean rollerMotorConnected,
      double rollerVelocityRPS,
      double rollerAppliedVolts,
      double rollerStatorCurrent,
      double rollerSupplyCurrentAmps,
      double rollerTempCelsius) {}

  record EE_CANRangeData(boolean canRangeConnected, boolean rangeIsTripped) {}

  default void updateInputs(ClawIOInputs inputs) {}

  default void setRollerVoltage(double voltage) {}

  default void setTorqueCurrent(double amps) {}

  default void setRollerVelocity(double velocity) {}

  default void setRollerPosition(double position) {}

  default void setRollerTargetPosition(double position) {}

  default boolean checkRollerStalled() {
    return false;
  }
}
