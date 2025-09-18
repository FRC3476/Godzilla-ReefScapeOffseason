package frc.robot.subsystems.end_effector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

  @AutoLog
  class EndEffectorIOInputs {
    public EE_PivotData pivotData = new EE_PivotData(false, 0, 0, 0, 0, 0);
    public EE_RollerData rollerData = new EE_RollerData(false, 0, 0, 0, 0, 0);
    public EE_CANRangeData canRangeData = new EE_CANRangeData(false, null);
  }

  record EE_PivotData(
      boolean pivotMotorConnected,
      double pivotPosition,
      double pivotAppliedVolts,
      double pivotTorqueCurrentAmps,
      double pivotSupplyCurrentAmps,
      double pivotTempCelsius) {}

  record EE_RollerData(
      boolean rollerMotorConnected,
      double rollerVelocityRPS,
      double rollerAppliedVolts,
      double rollerTorqueCurrentAmps,
      double rollerSupplyCurrentAmps,
      double rollerTempCelsius) {}

  record EE_CANRangeData(boolean canRangeConnected, Boolean rangeIsTripped) {}

  default void updateInputs(EndEffectorIOInputs inputs) {}

  default void setRollerVoltage(double voltage) {}

  default void setPivotVoltage(double voltage) {}

  default void setRollerVelocity(double velocity) {}

  default void setPivotTargetPosition(double position) {}

  default void setPivotPosition(double position) {}
}
