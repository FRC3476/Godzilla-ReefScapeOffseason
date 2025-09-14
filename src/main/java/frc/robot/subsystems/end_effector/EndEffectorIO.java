package frc.robot.subsystems.end_effector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

  @AutoLog
  class EndEffectorIOInputs {
    public PivotData pivotData = new PivotData(false, 0, 0, 0, 0, 0);
    public RollerData rollerData = new RollerData(false, 0, 0, 0, 0, 0);
    public CANRangeData canRangeData = new CANRangeData(false, null);
  }

  record PivotData(
      boolean pivotMotorConnected,
      double pivotPosition,
      double pivotAppliedVolts,
      double pivotTorqueCurrentAmps,
      double pivotSupplyCurrentAmps,
      double pivotTempCelsius) {}

  record RollerData(
      boolean rollerMotorConnected,
      double rollerVelocityRPS,
      double rollerAppliedVolts,
      double rollerTorqueCurrentAmps,
      double rollerSupplyCurrentAmps,
      double rollerTempCelsius) {}

  record CANRangeData(boolean canRangeConnected, Boolean rangeIsTripped) {}

  default void updateInputs(EndEffectorIOInputs inputs) {}

  default void setRollerVoltage(double voltage) {}

  default void setRollerVelocity(double velocity) {}

  default void setPivotTargetPosition(double position) {}

  default boolean checkRollerStalled() {
    return false;
  }
}
