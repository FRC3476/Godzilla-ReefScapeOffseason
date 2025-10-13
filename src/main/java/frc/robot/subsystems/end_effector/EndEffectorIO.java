package frc.robot.subsystems.end_effector;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

  @AutoLog
  class EndEffectorIOInputs {
    public EE_PivotData pivotData = new EE_PivotData(false, 0, 0, 0, 0, 0, 0);
  }

  record EE_PivotData(
      boolean pivotMotorConnected,
      double pivotPosition,
      double pivotAppliedVolts,
      double pivotTorqueCurrentAmps,
      double pivotSupplyCurrentAmps,
      double pivotTempCelsius,
      double pivotSetpoint
      // ,String pivotControlMode
      ) {}

  default void updateInputs(EndEffectorIOInputs inputs) {}

  default void setPivotVoltage(double voltage) {}

  default void setPivotPosition(DoubleSupplier position) {}

  default void updatePivotPIDFF(
      double kP,
      double kI,
      double kD,
      double kG,
      double kS,
      double velo,
      double accel,
      double jerk) {}

  default void setPivotZero() {}
}
