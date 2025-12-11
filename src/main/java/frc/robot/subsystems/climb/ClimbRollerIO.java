package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbRollerIO {

  @AutoLog
  class ClimbRollerIOInputs {
    public ClimbRollerData rollerData = new ClimbRollerData(false, 0, 0, 0, 0, 0);
  }

  record ClimbRollerData(
      boolean rollerMotorConnected,
      double rollerVelocityRPS,
      double rollerAppliedVolts,
      double rollerStatorCurrent,
      double rollerSupplyCurrentAmps,
      double rollerTempCelsius) {}

  default void updateInputs(ClimbRollerIOInputs inputs) {}

  default void setRollerVoltage(double voltage) {}

  default void setTorqueCurrent(double amps) {}

  default void setRollerVelocity(double velocity) {}

  default boolean checkRollerStalled() {
    return false;
  }
}
