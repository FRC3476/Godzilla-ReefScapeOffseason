package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public ElevatorIOData data =
        new ElevatorIOData(
            false, false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0);
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
      double rightSetPosition,
      double leftPosition,
      double leftAppliedVolts,
      double leftTorqueCurrentAmps,
      double leftSupplyCurrentAmps,
      double leftTempCelsius,
      double leftSetPosition,
      double extraPosition,
      double extraAppliedVolts,
      double extraTorqueCurrentAmps,
      double extraSupplyCurrentAmps,
      double extraTempCelsius,
      double extraSetPosition) {}

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setElevatorVoltage(double voltage) {}

  default void setElevatorTargetPosition(double position) {}

  default void setElevatorTargetPosition(DoubleSupplier supplier) {}

  default void setElevatorZero() {}

  default boolean checkMotorsStalled() {
    return false;
  }

  default void updateElevatorPIDFF(
      double kP,
      double kI,
      double kD,
      double kG,
      double kS,
      double velo,
      double accel,
      double jerk) {}
}
