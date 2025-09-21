package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

/* **********
 * COLLISION AVOIDANCE SOLUTION: Elevator class gets SS instance, defaul command sets to correct position (periodicially)
 ***********/

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private static Elevator elevatorSubsystem;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double setpoint;
  private boolean isZeroed = false;

  public static Elevator getInstance() {
    if (elevatorSubsystem == null) {
      elevatorSubsystem = new Elevator(new ElevatorIOReal());
    }
    return elevatorSubsystem;
  }

  public Elevator(ElevatorIO io) {
    this.io = io;
    System.out.println("====================Elevator Subsystem Online====================");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/Profile/TargetPosition", setpoint);
    Logger.recordOutput("Elevator/Profile/IsInTolerance", isInTolerance());
    Logger.recordOutput("Elevator/isZeroed", isZeroed);
    Logger.recordOutput("Elevator/foreignObjectDetected", checkForJam());
  }

  public void setTargetPosition(double position) {
    position =
        MathUtil.clamp(
            position,
            ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH,
            ElevatorConstants.ELEVATOR_MAX_SETPOINT_INCH);
    setpoint = position;
    io.setElevatorTargetPosition(position);
  }

  public boolean isInTolerance() {
    return MathUtil.isNear(
        setpoint, inputs.data.rightPosition(), ElevatorConstants.ELEVATOR_SETPOINT_TOLERANCE_INCH);
  }

  public double getTargetPosition() {
    return setpoint;
  }

  public Command moveToTargetPosition(double position) {
    return Commands.run(() -> this.setTargetPosition(position), this);
  }

  public double getCurrentPosition() {
    return inputs.data.rightPosition();
  }

  private boolean checkForJam() {
    if (io.checkMotorsStalled()
        && (MathUtil.isNear(0.0, getCurrentPosition(), ElevatorConstants.STALLED_TOLERANCE_INCHES)
            || !isZeroed)) {
      // false alarm, elevator is stalling at the bottom
      // make sure to run elevator down every time after turning it on
      io.setElevatorZero();
      isZeroed = true;
      return false;
    } else if (io.checkMotorsStalled()
        && getCurrentPosition()
            >= ElevatorConstants.ELEVATOR_MAX_SETPOINT_INCH
                - ElevatorConstants.STALLED_TOLERANCE_INCHES) {
      // false alarm, elevator is stalling at the top
      setTargetPosition(ElevatorConstants.ELEVATOR_MAX_SETPOINT_INCH);
      return false;
    } else {
      return io.checkMotorsStalled();
    }
  }

  public Command dejamElevator() {
    return Commands.runOnce(
        () -> setTargetPosition(getCurrentPosition() + ElevatorConstants.DEJAM_DISTANCE_INCHES));
  }

  public Trigger elevatorObjectTrigger = new Trigger(() -> checkForJam());
}
