package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Feeder feeder = Feeder.getInstance();

  private static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Intake/RollerVolts", 12.0);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean isCoralInIntake() {
    return inputs.canRangeData.tripped();
  }

  public Command intakeFWD() {
    return Commands.run(() -> this.io.setRollerVoltage(rollerIntakeVolts.get()), this);
  }

  public Command intakeRVS() {
    return Commands.run(() -> this.io.setRollerVoltage(-rollerIntakeVolts.get()), this);
  }

  public Command intakeSTOP() {
    return Commands.run(() -> this.io.setRollerVoltage(0), this);
  }

  public Trigger dejamTrigger = feeder.dejamTrigger;

  public Command feederDejam() {
    return feeder.feederDejam();
  }
}
