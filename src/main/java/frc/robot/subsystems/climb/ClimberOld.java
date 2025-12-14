package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RobotTime;
import org.littletonrobotics.junction.Logger;

public class ClimberOld extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

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

  // Tunable numbers for manual testing and gravity compensation
  private static final LoggedTunableNumber climberVolts =
      new LoggedTunableNumber("Climber/DeployVolts", 1);

  public ClimberOld(ClimberIO io) {
    this.io = io;
    io.setZero();
    limitSwitchLatching()
        .onTrue(Commands.runOnce(() -> setLimitSwitchState(LimitSwitchState.LATCHING)));
    limitSwitchLatched()
        .onTrue(Commands.runOnce(() -> setLimitSwitchState(LimitSwitchState.LATCHED)));
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    io.updateInputs(inputs);
    if (Timer.getFPGATimestamp() - latchedTimestamp
            > Constants.ClimbConstants.CLIMB_LATCHED_RESET_SECONDS
        && limitSwitchState == LimitSwitchState.LATCHED) {
      if (io.getLimitSwitch()) {
        setLimitSwitchState(LimitSwitchState.LATCHING);
      } else {
        setLimitSwitchState(LimitSwitchState.NONE);
      }
    }
    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Climber/ClimbState", climbState);
    Logger.recordOutput("Climber/LimitSwitchState", limitSwitchState);
    Logger.recordOutput(
        "Climber/currentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
    Logger.recordOutput(
        getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
  }

  public boolean isOK() {
    return inputs.data.motorConnected();
  }

  // public Command setClimbState(ClimbState state) {
  //   return Commands.runOnce(() -> climbState = state, this).asProxy().withName("Set Climb
  // State");
  // }

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

  public Command climbVoltOut() {
    return Commands.run(() -> this.io.runVolts(climberVolts.get()), this);
  }

  public Command climbDeploy() {
    return climbDeployToPosition(
            ClimbConstants.CLIMB_DEPLOY_POSITION, ClimbConstants.CLIMB_DEPLOY_VOLTAGE)
        .alongWith(Commands.runOnce(() -> ClimberOld.setClimbState(ClimbState.DEPLOYING)))
        .andThen(Commands.runOnce(() -> ClimberOld.setClimbState(ClimbState.DEPLOYED)));
  }

  public Command climbClimb() {
    return climbDeployToPosition(
            ClimbConstants.CLIMB_CLIMB_POSITION, ClimbConstants.CLIMB_CLIMB_VOLTAGE)
        .alongWith(Commands.runOnce(() -> ClimberOld.setClimbState(ClimbState.CLIMBING)))
        .andThen(Commands.runOnce(() -> ClimberOld.setClimbState(ClimbState.CLIMBED)));
  }


  public Command climbDeployToPosition(double position, double voltage) {
    if (inputs.data.positionRads() > position) {
      return climbSTOP();
    }
    return climbOut(voltage)
        .until(() -> inputs.data.positionRads() > position)
        .andThen(climbSTOP());
  }

  public Command climbDeployToPosition(double position) {
    if (inputs.data.positionRads() > position) {
      return climbSTOP();
    }
    return climbOut().until(() -> inputs.data.positionRads() > position).andThen(climbSTOP());
  }

  public Command climbOut(double voltage) {
    return Commands.run(() -> this.io.runVolts(voltage), this);
  }

  public Command climbOut() {
    return Commands.run(() -> this.io.runVolts(climberVolts.get()), this);
  }

  public Command climbSTOP() {
    return Commands.runOnce(() -> this.io.runVolts(0.0), this);
  }

  public Command climbRun() {
    return Commands.run(() -> this.io.runVolts(12), this).onlyWhile(climbFinished().negate());
  }

  public Trigger climbFinished() {
    return new Trigger(() -> this.io.checkClimbMotorStalled());
  }

  public Trigger limitSwitchLatching() {
    return new Trigger(
        () ->
            limitSwitchState == LimitSwitchState.NONE
                && io.getLimitSwitch()
                && climbState != ClimbState.STOWED);
  }

  public Trigger limitSwitchLatched() {
    return new Trigger(
        () ->
            limitSwitchState == LimitSwitchState.LATCHING
                && !io.getLimitSwitch()
                && climbState != ClimbState.STOWED);
  }
}