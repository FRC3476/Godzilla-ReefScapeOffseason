package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/* **********
 * COLLISION AVOIDANCE SOLUTION: Elevator class gets SS instance, defaul command sets to correct position (periodicially)
 ***********/

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private static final LoggedTunableNumber elevatorVolts =
      new LoggedTunableNumber("Elevator/TestVolts", 1.0);

  // Tunable numbers for manual testing
  private static final LoggedTunableNumber elevatorKP =
      new LoggedTunableNumber("Elevator/ElevatorKP", 0.0);
  private static final LoggedTunableNumber elevatorKI =
      new LoggedTunableNumber("Elevator/ElevatorKI", 0.0);
  private static final LoggedTunableNumber elevatorKD =
      new LoggedTunableNumber("Elevator/ElevatorKD", 0.0);
  private static final LoggedTunableNumber elevatorKG =
      new LoggedTunableNumber("Elevator/ElevatorKG", 0.0);
  private static final LoggedTunableNumber elevatorKS =
      new LoggedTunableNumber("Elevator/ElevatorKS", 0.0);

  private double setpoint;
  private boolean isZeroed = false;

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

    if (elevatorKP.hasChanged(hashCode())
        || elevatorKI.hasChanged(hashCode())
        || elevatorKD.hasChanged(hashCode())
        || elevatorKG.hasChanged(hashCode())
        || elevatorKS.hasChanged(hashCode())) {
      io.updateElevatorPIDFF(
          elevatorKP.get(), elevatorKI.get(), elevatorKD.get(), elevatorKG.get(), elevatorKS.get());
    }
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

  public Command moveToTargetPosition(DoubleSupplier positionSupplier) {
    return Commands.run(() -> this.setTargetPosition(positionSupplier.getAsDouble()), this);
  }

  public Command elevatorSTOP() {
    return Commands.run(() -> this.io.setElevatorVoltage(0), this);
  }

  public Command elevatorUP() {
    return Commands.run(() -> this.io.setElevatorVoltage(elevatorVolts.getAsDouble()), this);
  }

  public Command elevatorDWN() {
    return Commands.run(() -> this.io.setElevatorVoltage(-elevatorVolts.getAsDouble()), this);
  }

  public double getCurrentPosition() {
    return inputs.data.rightPosition();
  }

  private boolean checkForJam() {
    if (isHomingComplete()) {
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

  private boolean isHomingComplete() {
    // Check if homing is complete using the same logic as checkForJam for bottom detection
    if (io.checkMotorsStalled()
        && (MathUtil.isNear(0.0, getCurrentPosition(), ElevatorConstants.STALLED_TOLERANCE_INCHES)
            || !isZeroed)) {

      io.setElevatorZero();
      isZeroed = true;
      return true;
    }
    return false;
  }

  public Command manualSetElevatorZero() {
    isZeroed = true;
    return Commands.runOnce(() -> io.setElevatorZero(), this);
  }

  public Command dejamElevator() {
    return Commands.runOnce(
        () -> setTargetPosition(getCurrentPosition() + ElevatorConstants.DEJAM_DISTANCE_INCHES));
  }

  /** Command to home the elevator by running it slowly downward until it zeros. */
  public Command homeElevator() {
    return Commands.run(
            () -> this.io.setElevatorVoltage(ElevatorConstants.ELEVATOR_HOMING_VOLTAGE), this)
        .until(() -> isHomingComplete())
        .withTimeout(ElevatorConstants.HOMING_TIMEOUT_SECONDS)
        .finallyDo(() -> this.io.setElevatorVoltage(0.0))
        .withName("HomeElevator");
  }

  public Trigger elevatorObjectTrigger =
      new Trigger(() -> checkForJam()).debounce(ElevatorConstants.DEJAM_DEBOUNCE_SECONDS);
}
