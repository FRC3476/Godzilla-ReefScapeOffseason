package frc.robot.subsystems.end_effector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  private static EndEffector endEffectorSubsystem;

  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber("EndEffector/RollerVolts", 12.0);

  public static EndEffector getInstance() {
    if (endEffectorSubsystem == null) {
      endEffectorSubsystem = new EndEffector(new EndEffectorIOReal());
    }
    return endEffectorSubsystem;
  }

  public EndEffector(EndEffectorIO io) {
    this.io = io;
    System.out.println("====================EndEffector Subsystem Online====================");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);
  }

  public void setRollerVoltage(double voltage) {
    io.setRollerVoltage(voltage);
  }

  public boolean isCoralInEndeffector() {
    return inputs.canRangeData.rangeIsTripped() && inputs.canRangeData.canRangeConnected();
  }


  public double getCurrentPivotPosition() {
    return inputs.pivotData.pivotPosition();
  }

  public Command rollerFWD() {
    return Commands.run(() -> this.io.setRollerVoltage(rollerVolts.get()), this);
  }

  public Command rollerRVS() {
    return Commands.run(() -> this.io.setRollerVoltage(-rollerVolts.get()), this);
  }

  public Command rollerSTOP() {
    return Commands.run(() -> this.io.setRollerVoltage(0), this);
  }

  public Command moveToTargetRadian(double degree) {
    return Commands.run(() -> this.io.setPivotPosition(degree), this);
  }
}
