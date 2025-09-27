package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.CoralStateTracker;
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
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
    Logger.recordOutput("Feeder/JamDetected", checkForJam());

    CoralStateTracker.updateFeeder(isCoralInFeeder());

    // Update CoralStateTracker with feeder sensor data
    boolean feederSensorTriggered =
        inputs.canRangeData.tripped() && inputs.canRangeData.isSensorConnected();
    CoralStateTracker.updateFeeder(feederSensorTriggered);
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
    return io.checkMotorsStalled() && isCoralInFeeder();
  }

  public Trigger dejamTrigger =
      new Trigger(() -> checkForJam()).debounce(FeederConstants.DEJAM_DEBOUNCE_SECONDS);
}
