package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.FeederConstants;
import static edu.wpi.first.units.Units.Volts;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private static Feeder feederSubsystem;

  // SysId routine for characterization
  private final SysIdRoutine rollerSysId;

  public Feeder(FeederIO io) {
      this.io = io;
      
      // Configure SysId routine for roller motor
      rollerSysId =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  null,
                  null,
                  null,
                  state -> Logger.recordOutput("Feeder/SysIdState", state.toString())),
              new SysIdRoutine.Mechanism(
                  voltage -> io.setRollerVoltage(voltage.in(Volts)), null, this));
      
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

  // SysId characterization commands
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return rollerSysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return rollerSysId.dynamic(direction);
  }

  public Trigger dejamTrigger = new Trigger(() -> checkForJam()).debounce(FeederConstants.DEJAM_DEBOUNCE_SECONDS);
}
