package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.LoggedTunableNumber;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private static final LoggedTunableNumber climberIntakeVolts =
      new LoggedTunableNumber("ClimberVolts", 0);

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
    return Commands.run(() -> this.io.runVolts(climberIntakeVolts.get()), this);
  }

  public Command climbSTOP() {
    return Commands.run(() -> this.io.runVolts(0), this);
  }

  public Command climbRun() {
    return Commands.run(() -> this.io.runVolts(12), this).onlyWhile(climbFinished().negate());
  }

  public Trigger climbFinished() {
    return new Trigger(() -> this.io.checkClimbMotorStalled());
  }
}
