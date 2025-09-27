package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  
  // Tunable numbers for manual testing and gravity compensation
  private static final LoggedTunableNumber climberVolts =
      new LoggedTunableNumber("Climber/DeployVolts", 0);

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

  // Original commands (kept for compatibility)
  public Command climbDeploy() {
    return Commands.run(() -> this.io.runVolts(climberVolts.get()), this);
  }

  public Command climbSTOP() {
    return Commands.run(() -> this.io.runVolts(0.0), this);
  }

}
