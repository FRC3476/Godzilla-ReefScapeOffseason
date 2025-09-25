package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.Volts;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private static Elevator elevatorSubsystem;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private static final LoggedTunableNumber elevatorVolts =
      new LoggedTunableNumber("Elevator/TestVolts", 2.0);

  private double setpoint;
  private boolean isZeroed = false;

  // SysId routine for characterization
  private final SysIdRoutine elevatorSysId;

  public static Elevator getInstance() {
    if (elevatorSubsystem == null) {
      elevatorSubsystem = new Elevator(new ElevatorIOReal());
    }
    return elevatorSubsystem;
  }

  public Elevator(ElevatorIO io) {
    this.io = io;
    
    // Configure SysId routine for elevator motors
    elevatorSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> io.setElevatorVoltage(voltage.in(Volts)), null, this));
    
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

  public Command dejamElevator() {
    return Commands.runOnce(
        () -> setTargetPosition(getCurrentPosition() + ElevatorConstants.DEJAM_DISTANCE_INCHES));
  }
  
  // SysId characterization commands
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return elevatorSysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return elevatorSysId.dynamic(direction);
  }

  public Trigger elevatorJamTrigger = new Trigger(() -> checkForJam()).debounce(ElevatorConstants.DEJAM_DEBOUNCE_SECONDS);
}
