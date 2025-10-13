package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RobotTime;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  // Tunable numbers for manual testing and gravity compensation
  private static final LoggedTunableNumber climberVolts =
      new LoggedTunableNumber("Climber/DeployVolts", 1);

  public Climber(ClimberIO io) {
    this.io = io;
    io.setZero();
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    Logger.recordOutput(
        getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
  }

  public Command climbVoltOut() {
    return Commands.run(() -> this.io.runVolts(climberVolts.get()), this);
  }

  public Command climbDeploy() {
    return climbDeployToPosition(
        ClimbConstants.CLIMB_DEPLOY_POSITION, ClimbConstants.CLIMB_DEPLOY_VOLTAGE);
  }

  public Command climbClimb() {
    return climbDeployToPosition(
        ClimbConstants.CLIMB_CLIMB_POSITION, ClimbConstants.CLIMB_CLIMB_VOLTAGE);
  }

  public Command climbDeployToPosition(double position, double voltage) {
    if (inputs.data.positionRads() > position) {
      return climbSTOP();
    }
    return climbOut(voltage)
        .until(() -> inputs.data.positionRads() > position)
        .andThen(climbSTOP());
  }

  public Command climbDeployToPosition(double position) {
    if (inputs.data.positionRads() > position) {
      return climbSTOP();
    }
    return climbOut().until(() -> inputs.data.positionRads() > position).andThen(climbSTOP());
  }

  public Command climbOut(double voltage) {
    return Commands.run(() -> this.io.runVolts(voltage), this);
  }

  public Command climbOut() {
    return Commands.run(() -> this.io.runVolts(climberVolts.get()), this);
  }

  public Command climbSTOP() {
    return Commands.run(() -> this.io.runVolts(0.0), this);
  }

  public Command climbRun() {
    return Commands.run(() -> this.io.runVolts(12), this).onlyWhile(climbFinished().negate());
  }

  public Trigger climbFinished() {
    return new Trigger(() -> this.io.checkClimbMotorStalled());
  }
}
