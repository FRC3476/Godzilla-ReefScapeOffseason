package frc.robot.subsystems.feeder;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.util.RobotTime;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    this.io = io;
    System.out.println("====================Feeder Subsystem Online====================");
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);

    Logger.recordOutput("Feeder/JamDetected", checkForJam());
    Logger.recordOutput("Feeder/CoralInFeeder", isCoralInFeeder());
    CoralStateTracker.updateFeeder(isCoralInFeeder());

    Logger.recordOutput(
        getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    Logger.recordOutput(
        "Feeder/currentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
  }

  public boolean isCoralInFeeder() {
    return inputs.canRangeData.tripped() && inputs.canRangeData.isSensorConnected();
  }

  public void setRollerVoltage(double voltage) {
    io.setRollerVoltage(voltage);
  }

  public void setRollerVoltageReversed(double voltage) {
    io.setRollerVoltageReversed(voltage);
  }

  public boolean checkForJam() {
    return io.checkMotorsStalled();
  }

  public Trigger dejamTrigger =
      new Trigger(() -> checkForJam())
          .debounce(FeederConstants.DEJAM_DEBOUNCE_SECONDS, DebounceType.kBoth);

  public double whichMotorStalled() {
    return io.whichMotorStalled();
  }
}
