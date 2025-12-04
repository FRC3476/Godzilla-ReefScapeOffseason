package frc.robot.subsystems.end_effector;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.canDevice.CanRangeIO;
import frc.lib.subsystems.canDevice.CanRangeInputsAutoLogged;
import frc.lib.subsystems.real.ServoMotorSubsystem;
import frc.lib.subsystems.real.ServoMotorSubsystemConfig;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.EndEffectorConstants.ClawState;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.RobotTime;

public class Claw extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  private final RobotState robotState;
  private final CanRangeIO firstCANRangeIO;
  private final CanRangeIO secondCANRangeIO;

  private final CanRangeInputsAutoLogged firstRangeAutoLog = new CanRangeInputsAutoLogged();
  private final CanRangeInputsAutoLogged secondRangeAutoLog = new CanRangeInputsAutoLogged();

  private ClawState currentState = ClawState.NONE;
  private boolean firstSensorTriggered;
  private boolean secondSensorTriggered;

  public Claw(
      ServoMotorSubsystemConfig config,
      MotorIO io,
      RobotState robotState,
      CanRangeIO canRange,
      CanRangeIO canRange2) {
    super(config, new MotorInputsAutoLogged(), io);
    this.firstCANRangeIO = canRange;
    this.secondCANRangeIO = canRange2;
    this.robotState = robotState;
  }

  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    super.periodic();
    firstCANRangeIO.readInputs(firstRangeAutoLog);
    secondCANRangeIO.readInputs(secondRangeAutoLog);

    Logger.processInputs(getName() + "/firstCanRange", firstRangeAutoLog);
    Logger.processInputs(getName() + "/firstCanRange", secondRangeAutoLog);

    

    CoralStateTracker.updateFirstEndEffector(firstSensorTriggered);
    CoralStateTracker.updateSecondEndEffector(secondSensorTriggered);

    RobotState.setHasAlgae(hasAlgae());
  }

  public boolean isOK() {
    return firstRangeAutoLog.isConnected && secondRangeAutoLog.isConnected;
  }

  public boolean isClawScoring() {
    return currentState == ClawState.SCORING
        || currentState == ClawState.SCORING_L1
        || currentState == ClawState.SCORING_ALGAE;
  }

  public boolean isCoralInClaw() {
    // Use CoralStateTracker instead of individual sensor readings
    CoralStateTracker.CoralPosition position = CoralStateTracker.getCurrentPosition();
    return position == CoralStateTracker.CoralPosition.AT_FIRST_END_EFFECTOR
        || position == CoralStateTracker.CoralPosition.AT_SECOND_END_EFFECTOR
        || position == CoralStateTracker.CoralPosition.STAGED_IN_END_EFFECTOR;
  }

  public Trigger exhaustedCoral() {
    return new Trigger(
        () ->
            isClawScoring() && isCoralInClaw() && !firstSensorTriggered && !secondSensorTriggered);
  }

  public boolean isCoralAtFirstSensor() {
    return firstRangeAutoLog.isTripped && firstRangeAutoLog.isConnected;
  }

  public boolean isCoralAtSecondSensor() {
    return secondRangeAutoLog.isTripped && secondRangeAutoLog.isConnected;
  }

  public boolean hasAlgae() {
    return isMotorStalled(EndEffectorConstants.ROLLER_STALLED_CURRENT, EndEffectorConstants.ROLLER_STALLED_RPS) && !isCoralInClaw();
  }

  public Command clawDefault() {
    return Commands.run(
        () -> {
          CoralStateTracker.CoralPosition coralPosition = CoralStateTracker.getCurrentPosition();

          switch (this.currentState) {
            case NONE:
              break;
            case IDLE:
              break;
            case INTAKING_CORAL:
              // Check if coral is in end effector and automatically transition to HOLDING_CORAL
              if (coralPosition == CoralStateTracker.CoralPosition.AT_FIRST_END_EFFECTOR
                  || coralPosition == CoralStateTracker.CoralPosition.AT_SECOND_END_EFFECTOR
                  || coralPosition == CoralStateTracker.CoralPosition.STAGED_IN_END_EFFECTOR) {
                this.currentState = ClawState.HOLDING_CORAL;
              }
              break;
            case HOLDING_CORAL:
              break;
            case SCORING:
              // Transition to IDLE when coral is out of the end effector
              if (coralPosition == CoralStateTracker.CoralPosition.NONE) {
                this.currentState = ClawState.IDLE;
              }
              break;
            case SCORING_L1:
              if (coralPosition == CoralStateTracker.CoralPosition.NONE) {
                this.currentState = ClawState.IDLE;
              }
              break;
            case SCORING_ALGAE:
              // Transition to IDLE when coral is out of the end effector
              if (coralPosition == CoralStateTracker.CoralPosition.NONE) {
                this.currentState = ClawState.IDLE;
              }
              break;
            case ALGAE:
              if (RobotState.getSuperstructureTargetState() == SuperstructureState.STOW) {
                this.currentState = ClawState.IDLE;
              }
              break;
            default:
              this.currentState = ClawState.IDLE;
              break;
          }

          switch (this.currentState) {
            case NONE:
              break;
            case IDLE:
              setVoltage(() -> 0);
              break;
            case INTAKING_CORAL:
              if (coralPosition == CoralStateTracker.CoralPosition.NONE) {
                setVoltage(() -> 0);

              } else {
                setVoltage(() -> EndEffectorConstants.ROLLER_INTAKE_CORAL_VOLTS);
              }
              break;
            case HOLDING_CORAL:
              // move coral forward if at first sensor, backward if at second sensor, do nothing if
              // staged
              if (coralPosition == CoralStateTracker.CoralPosition.AT_FIRST_END_EFFECTOR) {
                setVoltage(() -> EndEffectorConstants.ROLLER_HOLDING_CORAL_VOLTS);
              } else if (coralPosition == CoralStateTracker.CoralPosition.AT_SECOND_END_EFFECTOR) {
                setVoltage(() -> -EndEffectorConstants.ROLLER_HOLDING_CORAL_VOLTS);
              } else if (coralPosition == CoralStateTracker.CoralPosition.STAGED_IN_END_EFFECTOR) {
                setVoltage(() -> 0);
              } else {
                setVoltage(() -> 0);
              }
              break;
            case SCORING:
              setVoltage(() -> EndEffectorConstants.ROLLER_SCORING_VOLTS);
              break;
            case SCORING_L1:
              setVoltage(() -> EndEffectorConstants.ROLLER_SCORING_L1_VOLTS);
              break;
            case SCORING_ALGAE:
              setVoltage(() -> EndEffectorConstants.ROLLER_SCORING_ALGAE_VOLTS);
              break;
            case ALGAE:
              setTorque(() -> EndEffectorConstants.CLAW_HOLD_ALGAE_AMPS);
              break;
            default:
              setVoltage(() -> 0);
              break;
          }
        },
        this);
  }

  public ClawState getClawState() {
    return currentState;
  }

  public Command setClawStateCommand(ClawState state) {
    return Commands.runOnce(() -> currentState = state)
        .onlyIf(
            () -> {
              boolean allowStateChange = true;

              if (state == ClawState.SCORING
                  && !RobotState.getSuperstructureState().isUprightScoringState()) {
                allowStateChange = false;
              }

              if (state == ClawState.SCORING_L1
                  && !RobotState.getSuperstructureState().isL1ScoringState()) {
                allowStateChange = false;
              }

              return allowStateChange;
            });
  }

  private Command setVoltage(DoubleSupplier voltageSupplier) {
    return new InstantCommand(() -> setVoltageImpl(voltageSupplier.getAsDouble()), this);
  }

  private Command setPosition(DoubleSupplier positionSupplier) {
    return new InstantCommand(() -> setPositionSetpointImpl(positionSupplier.getAsDouble()), this);
  }

  private Command setTorque(DoubleSupplier torqueSupplier) {
    return new InstantCommand(() -> setTorqueCurrentFOCImpl(torqueSupplier.getAsDouble()), this);
  }
}
