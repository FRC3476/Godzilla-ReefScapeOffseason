package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setPivotVoltage(double voltage) {
    io.setPivotVoltage(voltage);
  }

  public void setRollerVoltage(double voltage) {
    io.setRollerVoltage(voltage);
  }

  public void setPivotPosition(double positionRad) {
    io.setPivotPosition(positionRad);
  }

  public void stop() {
    io.stop();
  }

  public double getPivotPositionRad() {
    return inputs.pivotPositionRad;
  }

  public double getCANCoderPositionRad() {
    return inputs.canCoderPositionRad;
  }

  public boolean isCANRangeTripped() {
    return inputs.canRangeTripped;
  }

  public double getCANRangeDistanceMeters() {
    return inputs.canRangeDistanceMeters;
  }
}
