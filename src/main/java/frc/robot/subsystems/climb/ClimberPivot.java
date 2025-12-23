package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputs;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.real.ServoMotorSubsystem;
import frc.lib.subsystems.real.ServoMotorSubsystemConfig;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimberOld.ClimbState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RobotTime;

import org.littletonrobotics.junction.Logger;

public class ClimberPivot extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  private final RobotState robotState;
  private boolean isZeroed = false;
  private static final LoggedTunableNumber climbTestVolts =
      new LoggedTunableNumber("Climber/ClimberTestVolts", 1.0);
  protected final DigitalInput limitSwitch;

  public ClimberPivot(ServoMotorSubsystemConfig config, MotorIO io, RobotState robotState) {
    super(config, new MotorInputsAutoLogged(), io);
    limitSwitch = new DigitalInput(ClimbConstants.LIMIT_SWITCH_PIN);
    this.robotState = robotState;
    setDefaultCommand(
        motionMagicSetpointCommand(this::getPositionSetpointUnits)
            .withName(getName() + " Default Command Hold Position")
            .ignoringDisable(true));
  }

  public enum ClimbState {
    STOWED,
    DEPLOYING,
    DEPLOYED,
    CLIMBING,
    CLIMBED
  }

  public enum LimitSwitchState {
    NONE,
    LATCHING,
    LATCHED
  }

  private static ClimbState climbState = ClimbState.STOWED;

  public static LimitSwitchState limitSwitchState = LimitSwitchState.NONE;

  private static double latchedTimestamp = 0.0;

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    if (Timer.getFPGATimestamp() - latchedTimestamp
            > Constants.ClimbConstants.CLIMB_LATCHED_RESET_SECONDS
        && limitSwitchState == LimitSwitchState.LATCHED) {
      if (getLimitSwitch()) {
        setLimitSwitchState(LimitSwitchState.LATCHING);
      } else {
        setLimitSwitchState(LimitSwitchState.NONE);
      }
    }
    super.periodic();

    Logger.recordOutput(getName() + "/positionRotations", getCurrentPosition());
  }

  public boolean isOK() {
    return inputs.data.motorConnected();
  }

  public static void setClimbState(ClimbState state) {
    climbState = state;
  }

  public static ClimbState getClimbState() {
    return climbState;
  }

  public static void setLimitSwitchState(LimitSwitchState state) {
    limitSwitchState = state;
    if (state == LimitSwitchState.LATCHED) {
      latchedTimestamp = Timer.getFPGATimestamp();
    }
  }

  public Command climbDeploy() {
    return climbDeployToPosition(
            ClimbConstants.CLIMB_DEPLOY_POSITION, ClimbConstants.CLIMB_DEPLOY_VOLTAGE)
        .alongWith(Commands.runOnce(() -> setClimbState(ClimbState.DEPLOYING)))
        .andThen(Commands.runOnce(() -> setClimbState(ClimbState.DEPLOYED)));
  }

  public Command climbClimb() {
    return climbDeployToPosition(
            ClimbConstants.CLIMB_CLIMB_POSITION, ClimbConstants.CLIMB_CLIMB_VOLTAGE)
        .alongWith(Commands.runOnce(() -> setClimbState(ClimbState.CLIMBING)))
        .andThen(Commands.runOnce(() -> setClimbState(ClimbState.CLIMBED)));
  }

  public Command climbDeployToPosition(double position, double voltage) {
    if (inputs.unitPosition > position) {
      return climbSTOP();
    }
    return climbOut(voltage)
        .until(() -> inputs.unitPosition > position)
        .andThen(climbSTOP());
  }

  public Command climbDeployToPosition(double position) {
    if (inputs.unitPosition > position) {
      return climbSTOP();
    }
    return climbOut().until(() -> inputs.unitPosition > position).andThen(climbSTOP());
  }

  public Command climbOut(double voltage) {
    return voltageCommand(() -> voltage);
  }

  public Command climbOut() {
    return voltageCommand(() -> climbTestVolts.getAsDouble());
  }

  public Command climbSTOP() {
    return voltageCommand(() -> 0);
  }

  public boolean getLimitSwitch() {
    return !limitSwitch.get();
  }

  public Trigger limitSwitchLatching() {
    return new Trigger(
        () ->
            limitSwitchState == LimitSwitchState.NONE
                && getLimitSwitch()
                && climbState != ClimbState.STOWED);
  }

  public Trigger limitSwitchLatched() {
    return new Trigger(
        () ->
            limitSwitchState == LimitSwitchState.LATCHING
                && getLimitSwitch()
                && climbState != ClimbState.STOWED);
  }
}
