package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    // based on:
    // https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/subsystems/intake/SlamIO.java
    public PivotData pivotData = new PivotData(false, 0, 0, 0, 0, 0, 0, 0);
    public RollerData rollerData = new RollerData(false, 0, 0, 0, 0, 0);
    public CanCoderData canCoderData = new CanCoderData(false, 0, 0);
    public CanRangeData canRangeData = new CanRangeData(false, false, 0, 0);
  }

  /** pivot-related telemetry. */
  record PivotData(
      boolean isMotorConnected,
      double voltage,
      double supplyCurrent,
      double statorCurrent,
      double temperature,
      double velocityRPS,
      double positionRotation,
      double positionSetpoint) {}

  /** roller-related telemetry. */
  record RollerData(
      boolean isMotorConnected,
      double voltage,
      double supplyCurrent,
      double statorCurrent,
      double temperature,
      double velocityRPS) {}

  /** CANCoder telemetry. */
  record CanCoderData(boolean isSensorConnected, double positionRotations, double velocityRPS) {}

  /** CANRange telemetry. */
  record CanRangeData(
      boolean isSensorConnected, boolean tripped, double signalStrength, double distanceMeters) {}

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setPivotVoltage(double voltage) {}

  default void setRollerVoltage(double voltage) {}

  default void setPivotPosition(double positionRotations) {}

  default void updatePivotPIDFF(
      double kP,
      double kI,
      double kD,
      double kG,
      double kS,
      double velo,
      double accel,
      double jerk) {}

  default boolean checkRollerStalled() {
    return false;
  }

  default void setPivotZero() {}
}
