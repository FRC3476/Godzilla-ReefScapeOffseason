package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.*;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private static Elevator elevatorSubsystem;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private ElevatorState currentState = ElevatorState.DEFAULT;

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

  public void setZero() {
    io.setElevatorVoltage(-6);
    if (inputs.data.rightSupplyCurrentAmps() > ElevatorConstants.ELEVATOR_CURRENT_LIMIT_AMPS) {
      io.setElevatorZero();
      isZeroed = true;
    }
  }

  public boolean isInTolerance() {
    return MathUtil.isNear(
        setpoint,
        (inputs.data.rightPosition() + inputs.data.leftPosition() + inputs.data.extraPosition())
            / 3,
        ElevatorConstants.ELEVATOR_SETPOINT_TOLERANCE_INCH);
  }

  public double getTargetPosition() {
    return setpoint;
  }
}
