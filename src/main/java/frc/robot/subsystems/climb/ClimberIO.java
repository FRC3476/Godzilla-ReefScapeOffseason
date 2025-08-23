package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs{
        public ClimberIOData data = new ClimberIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    record ClimberIOData(
      boolean motorConnected,
      double positionRads,
      double velocityRadsPerSec,
      double appliedVoltage,
      double torqueCurrentAmps,
      double supplyVolts,
      double tempCelsius) {}
      
    default void runVolts(double volts){}
}
