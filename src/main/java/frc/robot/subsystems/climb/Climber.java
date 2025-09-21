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

  public Command climbMoveToTargetPosition(double position) {
    return Commands.run(() -> this.io.setClimbPosition(position), this);
  }

  public Command climbMoveToPreclimbPosition() {
    return Commands.run(() -> this.io.setClimbPosition(ClimbConstants.preclimbPosition), this);
  }

  public Command climbDeploy() {
    return Commands.run(() -> this.io.runVolts(climberIntakeVolts.get()), this);
  }

  public Command climbSTOP() {
    return Commands.run(() -> this.io.runVolts(0), this);
  }

  public Command climbRun() {
    return Commands.run(()-> this.io.runVolts(12),this).onlyWhile(climbFinished().negate());
    
  }
  
  public Trigger climbFinished(){
    return new Trigger(()-> this.io.checkClimbMotorStalled());

  }



}
