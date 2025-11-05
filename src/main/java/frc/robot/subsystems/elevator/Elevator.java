package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.ServoMotorSubsystemWithFollowers;
import frc.lib.subsystems.ServoMotorSubsystemWithFollowersConfig;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends ServoMotorSubsystemWithFollowers<MotorInputsAutoLogged, MotorIO> {
  private final RobotState robotState;
  private boolean isZeroed = false;
  private static final LoggedTunableNumber elevatorVolts =
      new LoggedTunableNumber("Elevator/TestVolts", 1.0);

  public Elevator(
      ServoMotorSubsystemWithFollowersConfig leadConfig,
      MotorIO leadIo,
      MotorIO[] followerIo,
      RobotState robotState) {
    super(
        leadConfig,
        new MotorInputsAutoLogged(),
        leadIo,
        new MotorInputsAutoLogged[] {new MotorInputsAutoLogged()},
        followerIo);
    this.robotState = robotState;
    setCurrentPositionAsZero();
    setDefaultCommand(
        motionMagicSetpointCommand(this::getPositionSetpointUnits)
            .withName(getName() + " Default Command Hold Position")
            .ignoringDisable(true));

    // In disabled, we can rezero if we see <0 position.
    if (DriverStation.isDisabled() && getCurrentPosition() < 0.0) {
      setCurrentPositionAsZero();
    }
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(getName() + "/positionMeters", getCurrentPosition());
  }

  public Command moveElevatorCommand(DoubleSupplier heightSupplier) {
    return positionSetpointUntilOnTargetCommand(
        () -> Units.inchesToMeters(heightSupplier.getAsDouble()),
        () -> Units.inchesToMeters(Constants.ElevatorConstants.ELEVATOR_SETPOINT_TOLERANCE_INCH));
  }

  public Command elevatorSTOP() {
    return dutyCycleCommand(() -> 0);
  }

  public Command elevatorUP() {
    return dutyCycleCommand(() -> elevatorVolts.getAsDouble());
  }

  public Command elevatorDWN() {
    return dutyCycleCommand(() -> -elevatorVolts.getAsDouble());
  }

  public double getMotorVelocityRPS() {
    return getCurrentVelocity() / Constants.ElevatorConstants.kElevatorUnitToRotorRatio;
  }

  private boolean isManualHomingComplete() {
    if (isMotorStalled(
        Constants.ElevatorConstants.STALLED_CURRENT,
        Constants.ElevatorConstants.STALLED_RPS
            * Constants.ElevatorConstants.kElevatorUnitToRotorRatio)) {
      setCurrentPosition(0);
      isZeroed = true;
      return true;
    }
    return false;
  }

  public Command manualHomeElevator() {
    return voltageCommand(() -> ElevatorConstants.ELEVATOR_HOMING_VOLTAGE)
        .until(() -> isManualHomingComplete())
        .withTimeout(ElevatorConstants.HOMING_TIMEOUT_SECONDS)
        .withName("ManualHomeElevator");
  }
}
