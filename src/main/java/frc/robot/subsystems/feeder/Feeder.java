package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.ElevatorIO;

import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private static Feeder feederSubsystem;

  public Feeder(FeederIO io) {
      this.io = io;
      System.out.println("====================Feeder Subsystem Online====================");
    }
  public static Feeder getInstance() {
    if (feederSubsystem == null) {
      feederSubsystem = new Feeder(new FeederIOReal());
    }
    return feederSubsystem;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
    Logger.recordOutput("Feeder/JamDetected", checkForJam());
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

  public Trigger dejamTrigger = new Trigger(() -> checkForJam()).debounce(FeederConstants.DEJAM_DEBOUNCE_SECONDS);
}
