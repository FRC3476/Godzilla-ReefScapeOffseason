package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    /** LEADER - Right motor telemetry. */
    public MotorData rightMotorData = new MotorData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public MotorData leftMotorData = new MotorData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public MotorData extraMotorData = new MotorData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  /** Motor telemetry data. */
  record MotorData(
      boolean isMotorConnected,
      double position,
      double appliedVolts,
      double torqueCurrentAmps,
      double supplyCurrentAmps,
      double tempCelsius,
      double setPosition) {}

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setElevatorVoltage(double voltage) {}

  default void setElevatorTargetPosition(double position) {}

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
