package frc.robot.subsystems.end_effector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

    @AutoLog
    class EndEffectorIOInputs {
        public EndEffectorIOData data =
            new EndEffectorIOData(
                false, false,
                0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0
            );
    }

    record EndEffectorIOData(
      boolean pivotMotorConnected,
      boolean rollerMotorConnected,
      double pivotPosition,
      double pivotAppliedVolts,
      double pivotTorqueCurrentAmps,
      double pivotSupplyCurrentAmps,
      double pivotTempCelsius,
      double rollerPosition,
      double rollerAppliedVolts,
      double rollerTorqueCurrentAmps,
      double rollerSupplyCurrentAmps,
      double rollerTempCelsius
    ) {}

    default void updateInputs(EndEffectorIOInputs inputs) {}

    default void setRollerVoltage(double voltage) {}

    default void setPivotTargetPosition(double position) {}
}