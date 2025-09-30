package frc.robot.subsystems.end_effector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.util.LoggedTunableNumber;

public class Claw extends SubsystemBase {

  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

  public Claw(ClawIO io) {
    this.io = io;
  }

  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber("EndEffector/RollerVolts", 1.0);

  public void periodic() {
    boolean firstSensorTriggered =
        inputs.firstCANRangeData.rangeIsTripped() && inputs.firstCANRangeData.canRangeConnected();
    boolean secondSensorTriggered =
        inputs.secondCANRangeData.rangeIsTripped() && inputs.secondCANRangeData.canRangeConnected();

    CoralStateTracker.updateFirstEndEffector(firstSensorTriggered);
    CoralStateTracker.updateSecondEndEffector(secondSensorTriggered);

    RobotState.setHasAlgae(hasAlgae());
  }

  public void setRollerVoltage(double voltage) {
    io.setRollerVoltage(voltage);
  }

  public boolean isCoralInEndeffector() {
    // Use CoralStateTracker instead of individual sensor readings
    CoralStateTracker.CoralPosition position = CoralStateTracker.getCurrentPosition();
    return position == CoralStateTracker.CoralPosition.AT_FIRST_END_EFFECTOR
        || position == CoralStateTracker.CoralPosition.AT_SECOND_END_EFFECTOR
        || position == CoralStateTracker.CoralPosition.STAGED_IN_END_EFFECTOR;
  }

  public boolean isCoralAtFirstSensor() {
    return inputs.firstCANRangeData.rangeIsTripped()
        && inputs.firstCANRangeData.canRangeConnected();
  }

  public boolean isCoralAtSecondSensor() {
    return inputs.secondCANRangeData.rangeIsTripped()
        && inputs.secondCANRangeData.canRangeConnected();
  }

  public boolean hasAlgae() {
    return io.checkRollerStalled() && !isCoralInEndeffector();
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
}
