package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  // Tunable numbers for manual testing and gravity compensation
  private static final LoggedTunableNumber climberVolts =
      new LoggedTunableNumber("Climber/DeployVolts", 1);

  private static Climber climberSubsystem;

  public static Climber getInstance() {
    if (climberSubsystem == null) {
      climberSubsystem = new Climber(new ClimberIOReal());
    }
    return climberSubsystem;
  }

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command climbVoltOut() {
    return Commands.runOnce(() -> this.io.runVolts(climberVolts.get()), this);
  }

  public Command climbDeploy() {
    return climbDeployToPosition(ClimbConstants.CLIMB_DEPLOY_POSITION);
  }

  public Command climbClimb() {
    return climbDeployToPosition(ClimbConstants.CLIMB_CLIMB_POSITION);
  }

  public Command climbDeployToPosition(double position) {
    if (inputs.data.positionRads() > position) {
      return climbSTOP();
    }
    return climbOut().until(() -> inputs.data.positionRads() > position).andThen(climbSTOP());
  }

  public Command climbOut() {
    return Commands.runOnce(() -> this.io.runVolts(climberVolts.get()), this);
  }

  public Command climbSTOP() {
    return Commands.runOnce(() -> this.io.runVolts(0.0), this);
  }

  public Command climbRun() {
    return Commands.run(() -> this.io.runVolts(12), this).onlyWhile(climbFinished().negate());
  }

  public Trigger climbFinished() {
    return new Trigger(() -> this.io.checkClimbMotorStalled());
  }
}
