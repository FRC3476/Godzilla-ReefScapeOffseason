package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Feeder feeder = Feeder.getInstance();

  private static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Intake/RollerVolts", 12.0);
  private static final LoggedTunableNumber rollerRejectVolts =
      new LoggedTunableNumber("Intake/RollerRejectVolts", 12.0); // Placeholder value
  private static final LoggedTunableNumber feederVolts =
      new LoggedTunableNumber("Feeder/RollerVolts", 12.0);

  private static Intake intakeSubsystem;

  public static Intake getInstance() {
    if (intakeSubsystem == null) {
      intakeSubsystem = new Intake(new IntakeIOReal());
    }
    return intakeSubsystem;
  }

  private IntakeState currentState = IntakeState.IDLE;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/JamDetected", checkForJam());

    // Update CoralStateTracker with intake sensor data
    boolean intakeSensorTriggered =
        inputs.canRangeData.tripped() && inputs.canRangeData.isSensorConnected();
    CoralStateTracker.updateIntake(intakeSensorTriggered);
  }

  public boolean isPivotAtSetpoint(double setpoint) {
    return Math.abs(inputs.pivotData.positionRad() - setpoint)
        < IntakeConstants.PIVOT_TOLERANCE_RAD;
  }

  public double getCurrentPivotPosition() {
    return inputs.pivotData.positionRad();
  }

  public boolean isCoralInIntake() {
    return inputs.canRangeData.tripped() && inputs.canRangeData.isSensorConnected();
  }

  private boolean checkForJam() {
    return io.checkRollerStalled() && isCoralInIntake();
  }

  public Trigger coralInIntakeTrigger() {
    return new Trigger(this::isCoralInIntake);
  }

  public Trigger rejectCoralTrigger() {
    return coralInIntakeTrigger()
        .and(
            () ->
                (CoralStateTracker.getCurrentPosition() == CoralStateTracker.CoralPosition.AT_FEEDER
                    || CoralStateTracker.getCurrentPosition()
                        == CoralStateTracker.CoralPosition.AT_FIRST_END_EFFECTOR
                    || CoralStateTracker.getCurrentPosition()
                        == CoralStateTracker.CoralPosition.AT_SECOND_END_EFFECTOR
                    || CoralStateTracker.getCurrentPosition()
                        == CoralStateTracker.CoralPosition.STAGED_IN_END_EFFECTOR));
  }

  public Command rejectCoralCommand() {
    return Commands.run(() -> this.io.setRollerVoltage(-rollerRejectVolts.get()), this);
  }

  public Command intakeFWD() {
    return Commands.runOnce(() -> this.io.setRollerVoltage(rollerIntakeVolts.get()), this);
  }

  public Command intakeRVS() {
    return Commands.runOnce(() -> this.io.setRollerVoltage(-rollerIntakeVolts.get()), this);
  }

  public Command intakeSTOP() {
    return Commands.runOnce(() -> this.io.setRollerVoltage(0), this);
  }

  public Command intakeDefault() {
    return Commands.run(
        () -> {
          switch (this.currentState) {
            case STOW:
              break;
            case INTAKE_L1:
              break;
            case INTAKE:
              // Check if coral is detected in feeder and automatically transition to IDLE
              if (feeder.isCoralInFeeder()) {
                this.currentState = IntakeState.IDLE;
              }
              break;
            case REJECT_CORAL:
              break;
            case HAND_OFF:
              break;
            case SCORING:
              break;
            case SCORING_PREP:
              break;
            case IDLE:
              break;
            default:
              this.currentState = IntakeState.IDLE;
              break;
          }

          // Execute motor commands based on current state
          switch (this.currentState) {
            case STOW:
              this.io.setPivotPosition(IntakeConstants.PIVOT_UP_POSITION);
              this.io.setRollerVoltage(0);
              feeder.setRollerVoltage(FeederConstants.FEEDER_STOP_VOLTS);
              break;
            case INTAKE_L1:
              this.io.setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION);
              this.io.setRollerVoltage(rollerIntakeVolts.get());
              this.io.setLvl1BlockerPosition(IntakeConstants.L1_BLOCKER_ENGAGED_POSITION);
              feeder.setRollerVoltage(FeederConstants.FEEDER_IN_VOLTS);
              break;
            case INTAKE:
              this.io.setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION);
              this.io.setRollerVoltage(rollerIntakeVolts.get());
              this.io.setLvl1BlockerPosition(IntakeConstants.L1_BLOCKER_DISENGAGED_POSITION);
              feeder.setRollerVoltage(FeederConstants.FEEDER_IN_VOLTS);
              break;
            case REJECT_CORAL:
              this.io.setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION);
              this.io.setRollerVoltage(-rollerIntakeVolts.get());
              feeder.setRollerVoltage(FeederConstants.FEEDER_OUT_VOLTS);
              break;
            case HAND_OFF:
              this.io.setRollerVoltage(0);
              feeder.setRollerVoltage(FeederConstants.FEEDER_IN_VOLTS);
              break;
            case SCORING:
              this.io.setPivotPosition(IntakeConstants.PIVOT_SCORING_POSITION);
              this.io.setRollerVoltage(IntakeConstants.ROLLER_SCORING_OUT_VOLTS);
              this.io.setLvl1BlockerPosition(IntakeConstants.L1_BLOCKER_ENGAGED_POSITION);
              feeder.setRollerVoltage(FeederConstants.FEEDER_STOP_VOLTS);
              break;
            case SCORING_PREP:
              this.io.setPivotPosition(IntakeConstants.SCORING_PREP_PIVOT_POSITION_RAD);
              this.io.setRollerVoltage(0);
              this.io.setLvl1BlockerPosition(IntakeConstants.L1_BLOCKER_ENGAGED_POSITION);
              feeder.setRollerVoltage(FeederConstants.FEEDER_STOP_VOLTS);
              break;
            case IDLE:
            default:
              this.io.setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION);
              this.io.setRollerVoltage(0);
              feeder.setRollerVoltage(FeederConstants.FEEDER_STOP_VOLTS);
              break;
          }
        },
        this);
  }

  public Command setIntakeState(IntakeState state) {
    return Commands.runOnce(() -> this.currentState = state, this);
  }

  public Command movePivotDown() {
    return Commands.runOnce(
        () -> this.io.setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION), this);
  }

  public Command rejectCoral() {
    return Commands.run(() -> this.io.setRollerVoltage(-rollerRejectVolts.get()), this);
  }

  public Command engageCoralL1() {
    return Commands.runOnce(
        () -> this.io.setLvl1BlockerPosition(IntakeConstants.L1_BLOCKER_CORAL_ENGAGED_POSITION));
  }

  public Command disengageCoralL1() {
    return Commands.runOnce(
        () -> this.io.setLvl1BlockerPosition(IntakeConstants.L1_BLOCKER_CORAL_DISENGAGED_POSITION));
  }

  public Trigger intakeJamTrigger =
      new Trigger(() -> checkForJam()).debounce(IntakeConstants.DEJAM_DEBOUNCE_SECONDS);

  public Trigger feederJamTrigger = feeder.dejamTrigger;

  public Command dejamFeeder() {
    return Commands.sequence(
        Commands.runOnce(() -> feeder.setRollerVoltageReversed(feederVolts.getAsDouble())),
        Commands.waitSeconds(FeederConstants.DEJAM_DURATION_SECONDS),
        Commands.runOnce(() -> feeder.setRollerVoltage(0.0)));
  }
}
