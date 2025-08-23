package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  class ElevatorIOInputs {
    public ElevatorIOData data =
        new ElevatorIOData(
            false, false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0);
  }

  record ElevatorIOData(
      boolean rightMotorConnected,
      boolean leftMotorConnected,
      boolean extraMotorConnected,
      double rightPosition,
      double rightAppliedVolts,
      double rightTorqueCurrentAmps,
      double rightSupplyCurrentAmps,
      double rightTempCelsius,
      double leftPosition,
      double leftAppliedVolts,
      double leftTorqueCurrentAmps,
      double leftSupplyCurrentAmps,
      double leftTempCelsius,
      double extraPosition,
      double extraAppliedVolts,
      double extraTorqueCurrentAmps,
      double extraSupplyCurrentAmps,
      double extraTempCelsius) {}

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setElevatorVoltage(double voltage) {}

  default void setElevatorTargetPosition(double position) {}

  default void setElevatorZero() {}

}
