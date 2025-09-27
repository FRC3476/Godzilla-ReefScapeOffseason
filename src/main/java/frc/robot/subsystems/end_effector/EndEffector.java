package frc.robot.subsystems.end_effector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  private static EndEffector endEffectorSubsystem;

  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber("EndEffector/RollerVolts", 12.0);
  private static final LoggedTunableNumber pivotTestVolts =
      new LoggedTunableNumber("EndEffector/PivotTestVolts", 2.0);

  public EndEffector(EndEffectorIO io) {
    this.io = io;
    System.out.println("====================EndEffector Subsystem Online====================");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);

    // Update CoralStateTracker with sensor data
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

  public Command rotatePivot(DoubleSupplier degreeSupplier) {
    return Commands.run(() -> this.io.setPivotPosition(degreeSupplier.getAsDouble()), this);
  }

  public Command pivotUP() {
    return Commands.run(() -> this.io.setPivotVoltage(-pivotTestVolts.get()), this);
  }

  public Command pivotDOWN() {
    return Commands.run(() -> this.io.setPivotVoltage(pivotTestVolts.get()), this);
  }

  public Command pivotSTOP() {
    return Commands.run(() -> this.io.setPivotVoltage(0), this);
  }
}
