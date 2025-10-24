package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RobotTime;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
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
      new LoggedTunableNumber("Elevator/ElevatorKP", ElevatorConstants.Tunable_ELEVATOR_kP);
  private static final LoggedTunableNumber elevatorKI =
      new LoggedTunableNumber("Elevator/ElevatorKI", ElevatorConstants.Tunable_ELEVATOR_kI);
  private static final LoggedTunableNumber elevatorKD =
      new LoggedTunableNumber("Elevator/ElevatorKD", ElevatorConstants.Tunable_ELEVATOR_kD);
  private static final LoggedTunableNumber elevatorKG =
      new LoggedTunableNumber("Elevator/ElevatorKG", ElevatorConstants.Tunable_ELEVATOR_kG);
  private static final LoggedTunableNumber elevatorKS =
      new LoggedTunableNumber("Elevator/ElevatorKS", ElevatorConstants.Tunable_ELEVATOR_kS);
  private static final LoggedTunableNumber elevatorVelo =
      new LoggedTunableNumber("Elevator/ElevatorVelo", ElevatorConstants.Tunable_ELEVATOR_Velo);
  private static final LoggedTunableNumber elevatorAccel =
      new LoggedTunableNumber("Elevator/ElevatorAccel", ElevatorConstants.Tunable_ELEVATOR_Accel);
  private static final LoggedTunableNumber elevatorJerk =
      new LoggedTunableNumber("Elevator/ElevatorJerk", ElevatorConstants.Tunable_ELEVATOR_Jerk);

  private double setpoint;
  private boolean isZeroed = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setElevatorZero();
    System.out.println("====================Elevator Subsystem Online====================");
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/TargetPosition", setpoint);
    // Logger.recordOutput("Elevator/Profile/IsInTolerance", isInTolerance());
    Logger.recordOutput("Elevator/isZeroed", isZeroed);
    // Logger.recordOutput("Elevator/foreignObjectDetected", checkForJam());

    if (elevatorKP.hasChanged(hashCode())
        || elevatorKI.hasChanged(hashCode())
        || elevatorKD.hasChanged(hashCode())
        || elevatorKG.hasChanged(hashCode())
        || elevatorKS.hasChanged(hashCode())
        || elevatorVelo.hasChanged(hashCode())
        || elevatorAccel.hasChanged(hashCode())
        || elevatorJerk.hasChanged(hashCode())) {
      io.updateElevatorPIDFF(
          elevatorKP.get(),
          elevatorKI.get(),
          elevatorKD.get(),
          elevatorKG.get(),
          elevatorKS.get(),
          elevatorVelo.get(),
          elevatorAccel.get(),
          elevatorJerk.get());
    }
    Logger.recordOutput(
        "Elevator/currentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
    Logger.recordOutput(
        getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
  }

  public void setTargetPositionCommand(double position) {
    position =
        MathUtil.clamp(
            position,
            ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH,
            ElevatorConstants.ELEVATOR_MAX_SETPOINT_INCH);
    setpoint = position;
    this.io.setElevatorTargetPosition(position);
  }

  @AutoLogOutput(key = "Elevator/InSetpointTolerance")
  public boolean isInToleranceSetpoint() {
    return MathUtil.isNear(
        setpoint, this.getCurrentPosition(), ElevatorConstants.ELEVATOR_SETPOINT_TOLERANCE_INCH);
  }

  public double getTargetPosition() {
    return setpoint;
  }

  public double getMotorVelocityRPS() {
    return inputs.rightMotorData.velocityRPS();
  }

  public Command moveElevatorCommand(DoubleSupplier heightSupplier) {
    return Commands.sequence(
        this.setTargetPositionCommand(heightSupplier), this.waitUntilTargetPositionCommand());
  }

  public Command setTargetPositionCommand(DoubleSupplier heightSupplier) {
    setpoint =
        MathUtil.clamp(
            heightSupplier.getAsDouble(),
            ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH,
            ElevatorConstants.ELEVATOR_MAX_SETPOINT_INCH);
    return Commands.runOnce(() -> this.io.setElevatorTargetPosition(setpoint), this);
  }

  public Command waitUntilTargetPositionCommand() {
    return Commands.waitUntil(() -> isInToleranceSetpoint());
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
    return inputs.rightMotorData.position();
  }

  private boolean checkForJam() {
    return false; // disabling check for Jam since it's untested. At least the homing works now

    // if (isHomingComplete()) {
    //   return false;
    // } else if (io.checkMotorsStalled()
    //     && getCurrentPosition()
    //         >= ElevatorConstants.ELEVATOR_MAX_SETPOINT_INCH
    //             - ElevatorConstants.STALLED_TOLERANCE_INCHES) {
    //   // false alarm, elevator is stalling at the top
    //   setTargetPositionCommand(ElevatorConstants.ELEVATOR_MAX_SETPOINT_INCH);
    //   return false;
    // } else {
    //   return io.checkMotorsStalled();
    // }
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

  private boolean isManualHomingComplete() {
    if (io.checkMotorsStalled()) {
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
        () ->
            setTargetPositionCommand(
                getCurrentPosition() + ElevatorConstants.DEJAM_DISTANCE_INCHES),
        this);
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
  /** Command to home the elevator by running it slowly downward until it zeros. */
  public Command manualHomeElevator() {
    return Commands.run(
            () -> this.io.setElevatorVoltage(ElevatorConstants.ELEVATOR_HOMING_VOLTAGE), this)
        .until(() -> isManualHomingComplete())
        .withTimeout(ElevatorConstants.HOMING_TIMEOUT_SECONDS)
        .finallyDo(() -> this.io.setElevatorVoltage(0.0))
        .withName("ManualHomeElevator");
  }

  public Trigger elevatorObjectTrigger =
      new Trigger(() -> checkForJam()).debounce(ElevatorConstants.DEJAM_DEBOUNCE_SECONDS);
}
