package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private double intakeRollerVolts = 0.0;
  private double intakePivotVolts = 0.0;
  private double intakelvl1BlockerVolts = 0.0;

  private DCMotorSim intakeRollerMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.01, Constants.IntakeConstants.ROLLER_GEAR_RATIO),
          DCMotor.getKrakenX60(1));

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeRollerMotor.update(Constants.LOOP_PERIOD_SECS);
    inputs.rollerData =
        new RollerData(
            true,
            intakeRollerVolts,
            Math.abs(intakeRollerMotor.getCurrentDrawAmps()),
            Math.abs(intakeRollerMotor.getCurrentDrawAmps()),
            0.0,
            intakeRollerMotor.getAngularVelocityRadPerSec());
  }

  @Override
  public void setRollerVoltage(double voltage) {
    intakeRollerVolts = MathUtil.clamp(voltage, -12, 12);
    intakeRollerMotor.setInputVoltage(intakeRollerVolts);
  }

  @Override
  public void setPivotVoltage(double voltage) {}

  @Override
  public void setLvl1BlockerVoltage(double voltage) {}

  @Override
  public void setPivotPosition(double positionRad) {}

  @Override
  public void setLvl1BlockerPosition(double positionRad) {}
}
