package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.real.ServoMotorSubsystem;
import frc.lib.subsystems.real.ServoMotorSubsystemConfig;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ClimbRoller extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  private final RobotState robotState;
  private static final LoggedTunableNumber rollerTestVolts =
      new LoggedTunableNumber("ClimbRoller/TunableVolts", 1.0);
  private static final LoggedTunableNumber rollerTestHoldingCageAmps =
      new LoggedTunableNumber("ClimbRoller/HoldingCageAmps", ClimbConstants.ROLLER_HOLD_CAGE_AMPS);

  private boolean climbing = false;

  public enum ClimbState {
    STOWED,
    DEPLOYING,
    DEPLOYED,
    CLIMBING,
    CLIMBED
  }

  private static ClimbState climbState = ClimbState.STOWED;

  public enum LimitSwitchState {
    NONE,
    LATCHING,
    LATCHED
  }

  public static LimitSwitchState limitSwitchState = LimitSwitchState.NONE;

  public boolean checkRollerStalled() {
    return isMotorStalled(
      ClimbConstants.ROLLER_STALLED_CURRENT,
      ClimbConstants.ROLLER_STALLED_RPS);
  }

  public ClimbRoller(ServoMotorSubsystemConfig config, MotorIO io, RobotState robotState) {
    super(config, new MotorInputsAutoLogged(), io);
    this.robotState = robotState;
    setDefaultCommand(
        motionMagicSetpointCommand(this::getPositionSetpointUnits)
            .withName(getName() + " Default Command Hold Position")
            .ignoringDisable(true));
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(getName() + "/positionRotations", getCurrentPosition());
  }

  public void setRollerVoltage(double voltage) {
    io.setVoltageOutput(voltage);
  }

  public static void setClimbState(ClimbState deploying) {
    climbState = deploying;
  }

  public static ClimbState getClimbState() {
    return climbState;
  }

  public Command rollerFWD() {
    return voltageCommand(() -> rollerTestVolts.getAsDouble());
  }

  public Command rollerRVS() {
    return voltageCommand(() -> -rollerTestVolts.getAsDouble());
  }

  public Command rollerSTOP() {
    return voltageCommand(() -> 0);
  }

  public Command holdCage() {
    return setTorqueCurrentFOC(() -> rollerTestHoldingCageAmps.get());
  }

  public boolean getClimbing() {
    return climbing;
  }

  public boolean hasCage() {
    return ClimbRoller.limitSwitchState == ClimbRoller.limitSwitchState.LATCHED
        && climbing
        && checkRollerStalled();
  }

  public void setClimbing(boolean climbing) {
    this.climbing = climbing;
  }
}
