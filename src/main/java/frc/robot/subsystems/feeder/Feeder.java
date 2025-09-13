package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeederConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private static Feeder feeder = null;

  private static final LoggedTunableNumber feederVolts =
      new LoggedTunableNumber("Feeder/RollerVolts", 12.0);

  public Feeder(FeederIO io) {
    this.io = io;
  }

  public static Feeder getInstance() {
    if (feeder == null) {
      feeder = new Feeder(new FeederIOReal());
    }
    return feeder;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
    checkForJam();
  }

  public boolean isCoralInFeeder() {
    return inputs.canRangeData.tripped();
  }

  public void setRollerVoltage(double voltage) {
    io.setRollerVoltage(voltage);
  }

  public boolean checkForJam() {
    return io.checkMotorsStalled() && isCoralInFeeder();
  }

  public Command feederDejam() {
    return Commands.sequence(
        Commands.runOnce(() -> io.dejamCoral()),
        Commands.waitSeconds(FeederConstants.DEJAM_DURATION_SECONDS),
        Commands.runOnce(() -> io.finishDejam()));
  }

  public Trigger dejamTrigger = new Trigger(() -> checkForJam());
}
