package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private static Elevator elevatorSubsystem;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private double setpoint;
  private boolean isZeroed = false;

  public static Elevator getInstance() {
    if (elevatorSubsystem == null) {
      elevatorSubsystem = new Elevator(new ElevatorIOTalonFX());
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
  }

  public void setTargetPosition(double position) {
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

  private boolean checkForJam() {
    if (io.checkMotorsStalled()
        && (MathUtil.isNear(0.0, getTargetPosition(), ElevatorConstants.STALLED_ZERO_MARGIN_INCHES)
            || !isZeroed)) {
      io.setElevatorZero();
      isZeroed = true;
      return false;
    } else {
      return io.checkMotorsStalled();
    }
  }

  public Command dejamElevator() {
    return Commands.run(
        () -> setTargetPosition(getTargetPosition() + ElevatorConstants.DEJAM_DISTANCE_INCHES));
  }

  public Trigger elevatorObjectTrigger = new Trigger(() -> checkForJam());
}
