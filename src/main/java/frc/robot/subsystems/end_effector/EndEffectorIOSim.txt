package frc.robot.subsystems.end_effector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class EndEffectorIOSim implements EndEffectorIO {
  private double rollerVolts = 0.0;

  private DCMotorSim rollerMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.01, Constants.EndEffectorConstants.ALGAE_GEAR_RATIO),
          DCMotor.getKrakenX60(1));

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    rollerMotor.update(Constants.LOOP_PERIOD_SECS);
    inputs.rollerData =
        new EE_RollerData(
            true,
            rollerMotor.getAngularVelocityRadPerSec(),
            rollerVolts,
            Math.abs(rollerMotor.getCurrentDrawAmps()),
            Math.abs(rollerMotor.getCurrentDrawAmps()),
            0.0);
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollerVolts = MathUtil.clamp(voltage, -12, 12);
    rollerMotor.setInputVoltage(rollerVolts);
  }

  @Override
  public void setRollerVelocity(double velocity) {}

  @Override
  public void setPivotTargetPosition(double position) {}
}
