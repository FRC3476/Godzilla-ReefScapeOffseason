package frc.robot.subsystems.end_effector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.EndEffectorConstants.ClawState;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RobotTime;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {

  private final ClawIO io;
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber(
          "Claw/RollerVolts", 1.0); // It was already set to 1.0 and used in rollerFWD and rollerRVS
  private static final LoggedTunableNumber rollerIntakeCoralVolts =
      new LoggedTunableNumber(
          "Claw/RollerIntakeCoralVolts", EndEffectorConstants.ROLLER_INTAKE_CORAL_VOLTS);
  private static final LoggedTunableNumber rollerHoldingCoralVolts =
      new LoggedTunableNumber(
          "Claw/RollerHoldingCoralVolts", EndEffectorConstants.ROLLER_HOLDING_CORAL_VOLTS);
  private static final LoggedTunableNumber rollerScoringVolts =
      new LoggedTunableNumber("Claw/RollerScoringVolts", EndEffectorConstants.ROLLER_SCORING_VOLTS);
  private static final LoggedTunableNumber rollerScoringL1Volts =
      new LoggedTunableNumber(
          "Claw/RollerScoringL1Volts", EndEffectorConstants.ROLLER_SCORING_L1_VOLTS);
  private static final LoggedTunableNumber rollerScoringAlgaeVolts =
      new LoggedTunableNumber(
          "Claw/RollerScoringL1Volts", EndEffectorConstants.ROLLER_SCORING_ALGAE_VOLTS);

  private ClawState currentState = ClawState.NONE;
  private boolean firstSensorTriggered;
  private boolean secondSensorTriggered;

  public Claw(ClawIO io) {
    this.io = io;
  }

  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);
    Logger.recordOutput("Claw/CurrentState", currentState);

    firstSensorTriggered =
        inputs.firstCANRangeData.rangeIsTripped() && inputs.firstCANRangeData.canRangeConnected();
    secondSensorTriggered =
        inputs.secondCANRangeData.rangeIsTripped() && inputs.secondCANRangeData.canRangeConnected();

    CoralStateTracker.updateFirstEndEffector(firstSensorTriggered);
    CoralStateTracker.updateSecondEndEffector(secondSensorTriggered);

    RobotState.setHasAlgae(hasAlgae());

    Logger.recordOutput(
        getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    Logger.recordOutput(
        "Claw/currentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
  }

  public void setRollerVoltage(double voltage) {
    io.setRollerVoltage(voltage);
  }

  public boolean isClawScoring() {
    return currentState == ClawState.SCORING || currentState == ClawState.SCORING_L1;
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
    return inputs.firstCANRangeData.rangeIsTripped()
        && inputs.firstCANRangeData.canRangeConnected();
  }

  public boolean isCoralAtSecondSensor() {
    return inputs.secondCANRangeData.rangeIsTripped()
        && inputs.secondCANRangeData.canRangeConnected();
  }

  public boolean hasAlgae() {
    return io.checkRollerStalled() && !isCoralInClaw();
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
              // if (coralPosition == CoralStateTracker.CoralPosition.NONE) {
              //   this.currentState = ClawState.IDLE;
              // }
              break;
            case SCORING_L1:
              if (coralPosition == CoralStateTracker.CoralPosition.NONE) {
                this.currentState = ClawState.IDLE;
              }
              break;
            case SCORING_ALGAE:
              // Transition to IDLE when coral is out of the end effector
              // if (coralPosition == CoralStateTracker.CoralPosition.NONE) {
              //   this.currentState = ClawState.IDLE;
              // }
              break;
            case ALGAE:
              if (RobotState.getSuperstructureState() == SuperstructureState.STOW) {
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
              this.io.setRollerVoltage(0);
              break;
            case INTAKING_CORAL:
              this.io.setRollerVoltage(rollerIntakeCoralVolts.get());
              break;
            case HOLDING_CORAL:
              // move coral forward if at first sensor, backward if at second sensor, do nothing if
              // staged
              if (coralPosition == CoralStateTracker.CoralPosition.AT_FIRST_END_EFFECTOR) {
                this.io.setRollerVoltage(rollerHoldingCoralVolts.get());
              } else if (coralPosition == CoralStateTracker.CoralPosition.AT_SECOND_END_EFFECTOR) {
                this.io.setRollerVoltage(-rollerHoldingCoralVolts.get());
              } else if (coralPosition == CoralStateTracker.CoralPosition.STAGED_IN_END_EFFECTOR) {
                this.io.setRollerVoltage(0);
              } else {
                this.io.setRollerVoltage(0);
              }
              break;
            case SCORING:
              this.io.setRollerVoltage(rollerScoringVolts.get());
              break;
            case SCORING_L1:
              this.io.setRollerVoltage(rollerScoringL1Volts.get());
              break;
            case SCORING_ALGAE:
              this.io.setRollerVoltage(rollerScoringAlgaeVolts.get());
              break;
            case ALGAE:
              this.io.setTorqueCurrent(EndEffectorConstants.CLAW_HOLD_ALGAE_AMPS);
              break;
            default:
              this.io.setRollerVoltage(0);
              break;
          }
        },
        this);
  }

  public Command setClawStateCommand(ClawState state) {
    return Commands.runOnce(() -> currentState = state);
  }

  public Command rollerFWD() {
    return Commands.runOnce(() -> this.io.setRollerVoltage(rollerVolts.get()), this);
  }

  public Command rollerRVS() {
    return Commands.runOnce(() -> this.io.setRollerVoltage(-rollerVolts.get()), this);
  }

  public Command rollerSTOP() {
    return Commands.runOnce(() -> this.io.setRollerVoltage(0), this);
  }

  public Command holdAlgae() {
    return Commands.runOnce(
        () -> this.io.setTorqueCurrent(EndEffectorConstants.CLAW_HOLD_ALGAE_AMPS), this);
  }
}
