package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    public Command climbDeploy (){
        return Commands.run(() -> this.io.runVolts(ClimberIntakeVolts.get()), this);
    }

    public Command intakeSTOP() {
    return Commands.run(() -> this.io.runVoltage(0), this);
  }
}


