package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog

  class ElevatorIOInputs {
    /** LEADER - Right motor telemetry. */
    public RightMotorData rightMotorData = new RightMotorData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public LeftMotorData leftMotorData = new LeftMotorData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public ExtraMotorData extraMotorData = new ExtraMotorData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }


  /** motor telemetry data. */
  record RightMotorData(
    boolean isMotorConnected,
    double position,
    double appliedVolts,
    double torqueCurrentAmps,
    double supplyCurrentAmps,
    double tempCelsius,
    double setPosition) {}

  /** motor telemetry data. */
  record LeftMotorData(
    boolean isMotorConnected,
    double position,
    double appliedVolts,
    double torqueCurrentAmps,
    double supplyCurrentAmps,
    double tempCelsius,
    double setPosition) {}

  /** motor telemetry data. */
  record ExtraMotorData(
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
