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

    public double pivotVoltage;

    public double pivotSupplyCurrent;

    public double pivotStatorCurrent;

    public double pivotTemperature;

    public double pivotVelocityRPS;

    public double pivotPositionRad;

    public double rollerVoltage;

    public double rollerStatorCurrent;

    public double rollerSupplyCurrent;

    public double rollerTemperature;

    public double rollerVelocityRPS;

    public double canCoderPositionRad;

    public double canCoderVelocityRPS;

    public double canRangeSignalStrength;

    public Boolean canRangeTripped;

    public double canRangeDistanceMeters;
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

  void updateInputs(IntakeIOInputs inputs);

  void stop();

  void setPivotPosition(double positionRad);

  void setRollerVoltage(double voltage);

  void setPivotVoltage(double voltage);
  
}
