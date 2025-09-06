package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private static final LoggedTunableNumber climberIntakeVolts = new LoggedTunableNumber("ClimbVolts", 12.0);

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  
  public Command climbDeploy (){
    return Commands.run(() -> this.io.runVolts(ClimberIntakeVolts.get()), this);
}

public Command intakeSTOP() {
    return Commands.run(() -> this.io.runVoltage(0), this);
}


}
