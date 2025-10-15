package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RobotTime;
import org.littletonrobotics.junction.Logger;

public class ClimbRoller extends SubsystemBase {

  private final ClimbRollerIO io;
  private final ClimbRollerIOInputsAutoLogged inputs = new ClimbRollerIOInputsAutoLogged();

  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber(
        "ClimbRoller/Volts", 1.0); // It was already set to 1.0 and used in rollerFWD and rollerRVS
  private static final LoggedTunableNumber rollerHoldingCageVolts =
      new LoggedTunableNumber(
        "CLimbRoller/HoldingCageVolts", ClimbConstants.ROLLER_HOLDING_CAGE_AMPS); 
  private static final LoggedTunableNumber rollerBackOutvolts =
      new LoggedTunableNumber(
        "ClimbRoller/ScoringVolts", ClimbConstants.ROLLER_BACKOUT_VOLTS); 

  public ClimbRoller(ClimbRollerIO io) {
    this.io = io;
  }

  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    io.updateInputs(inputs);
    Logger.processInputs("ClimbRoller", inputs);

    Logger.recordOutput(
        getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    Logger.recordOutput(
        "ClimbRoller/currentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
  }

  public void setRollerVoltage(double voltage) {
    io.setRollerVoltage(voltage);
  }

  public Command rollerFWD() {
    return Commands.runOnce(() -> this.io.setRollerVoltage(rollerVolts.get()), this);
  }

  public Command rollerRVS() {
    return Commands.runOnce(() -> this.io.setRollerVoltage(-rollerVolts.get()), this);
  }

  public Command rollerSTOP() {
    return Commands.runOnce(() -> this.io.setRollerVoltage(0), this);
  }

  public Command holdCage() {
    return Commands.runOnce(
        () -> this.io.setTorqueCurrent(ClimbConstants.ROLLER_HOLDING_CAGE_AMPS), this); 
  }
}
