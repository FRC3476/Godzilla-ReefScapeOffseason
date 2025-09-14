package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();


  private static Feeder feederSubsystem;

  public static Feeder getInstance() {
    if (feederSubsystem == null) {
      feederSubsystem = new Feeder(new FeederIOReal());
    }
    return feederSubsystem;
  }

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public boolean isCoralInFeeder() {
    return inputs.canRangeData.tripped() && inputs.canRangeData.isSensorConnected();
  }

  public void setRollerVoltage(double voltage) {
    io.setRollerVoltage(voltage);
  }
}
