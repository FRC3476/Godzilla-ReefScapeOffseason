package frc.robot.subsystems.end_effector;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {

  @AutoLog
  class ClawIOInputs {
    public EE_RollerData rollerData = new EE_RollerData(false, 0, 0, 0, 0, 0);
    public EE_CANRangeData firstCANRangeData = new EE_CANRangeData(false, false);
    public EE_CANRangeData secondCANRangeData = new EE_CANRangeData(false, false);
  }

  record EE_RollerData(
      boolean rollerMotorConnected,
      double rollerVelocityRPS,
      double rollerAppliedVolts,
      double rollerTorqueCurrentAmps,
      double rollerSupplyCurrentAmps,
      double rollerTempCelsius) {}

  record EE_CANRangeData(boolean canRangeConnected, boolean rangeIsTripped) {}

  default void setRollerVoltage(double voltage) {}

  default void setTorqueCurrent(double amps) {}

  default void setRollerVelocity(double velocity) {}

  default boolean checkRollerStalled() {
    return false;
  }
}
