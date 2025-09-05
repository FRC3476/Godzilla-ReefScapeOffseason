package frc.robot.subsystems.end_effector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

  @AutoLog
  class EndEffectorIOInputs {
    public EndEffectorIOData data =
        new EndEffectorIOData(
            false, false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
  }

  record EndEffectorIOData(
      boolean pivotMotorConnected,
      boolean rollerMotorConnected,
      boolean canRangeConnected,
      double pivotPosition,
      double pivotAppliedVolts,
      double pivotTorqueCurrentAmps,
      double pivotSupplyCurrentAmps,
      double pivotTempCelsius,
      double rollerPosition,
      double rollerAppliedVolts,
      double rollerTorqueCurrentAmps,
      double rollerSupplyCurrentAmps,
      double rollerTempCelsius,
      Boolean rangeIsTripped) {}

  default void updateInputs(EndEffectorIOInputs inputs) {}

  default void setRollerVoltage(double voltage) {}

  default void setRollerVelocity(double velocity) {}

  default void setPivotTargetPosition(double position) {}
}
