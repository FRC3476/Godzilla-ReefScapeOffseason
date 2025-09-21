package frc.robot.subsystems.end_effector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.CoralStateTracker;
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
    
    // Update CoralStateTracker with sensor data
    boolean firstSensorTriggered = inputs.firstCANRangeData.rangeIsTripped() != null 
        && inputs.firstCANRangeData.rangeIsTripped() 
        && inputs.firstCANRangeData.canRangeConnected();
    boolean secondSensorTriggered = inputs.secondCANRangeData.rangeIsTripped() != null 
        && inputs.secondCANRangeData.rangeIsTripped() 
        && inputs.secondCANRangeData.canRangeConnected();
        
    CoralStateTracker.updateFirstEndEffector(firstSensorTriggered);
    CoralStateTracker.updateSecondEndEffector(secondSensorTriggered);
  }

  public void setRollerVoltage(double voltage) {
    io.setRollerVoltage(voltage);
  }

  public boolean isCoralInEndeffector() {
    // Use CoralStateTracker instead of individual sensor readings
    CoralStateTracker.CoralPosition position = CoralStateTracker.getCurrentPosition();
    return position == CoralStateTracker.CoralPosition.AT_FIRST_END_EFFECTOR ||
           position == CoralStateTracker.CoralPosition.AT_SECOND_END_EFFECTOR ||
           position == CoralStateTracker.CoralPosition.STAGED_IN_END_EFFECTOR;
  }

  public boolean isCoralAtFirstSensor() {
    return inputs.firstCANRangeData.rangeIsTripped() != null 
        && inputs.firstCANRangeData.rangeIsTripped() 
        && inputs.firstCANRangeData.canRangeConnected();
  }

  public boolean isCoralAtSecondSensor() {
    return inputs.secondCANRangeData.rangeIsTripped() != null 
        && inputs.secondCANRangeData.rangeIsTripped() 
        && inputs.secondCANRangeData.canRangeConnected();
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
