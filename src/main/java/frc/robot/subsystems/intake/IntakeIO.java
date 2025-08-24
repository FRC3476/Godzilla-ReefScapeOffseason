package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    // based on: https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/subsystems/intake/SlamIO.java
    public PivotData pivot = new PivotData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public RollerData roller = new RollerData(0.0, 0.0, 0.0, 0.0, 0.0);

    public CanCoderData canCoder = new CanCoderData(0.0, 0.0);

    public CanRangeData canRange = new CanRangeData(false, 0.0, 0.0);
  }

  /**
   *  pivot-related telemetry.
   */
  record PivotData(
      double voltage,
      double supplyCurrent,
      double statorCurrent,
      double temperature,
      double velocityRPS,
      double positionRad) {}

  /**
   * roller-related telemetry.
   */
  record RollerData(
      double voltage, double supplyCurrent, double statorCurrent, double temperature, double velocityRPS) {}

  /** CANCoder telemetry. */
  record CanCoderData(double positionRad, double velocityRPS) {}

  /** CANRange telemetry. */
  record CanRangeData(boolean tripped, double signalStrength, double distanceMeters) {}
  
  /** Run the pivot motor */
  default void runPivotVolts(double volts) {}

}
