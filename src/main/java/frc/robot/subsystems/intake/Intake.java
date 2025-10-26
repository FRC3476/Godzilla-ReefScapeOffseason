package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.RobotState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.CoralStateTracker.CoralPosition;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RobotTime;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private Feeder feeder;

  private static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Intake/RollerVolts", 12.0);
  private static final LoggedTunableNumber rollerRejectVolts =
      new LoggedTunableNumber("Intake/RollerRejectVolts", 12.0); // Placeholder value
  private static final LoggedTunableNumber feederVolts =
      new LoggedTunableNumber("Feeder/RollerVolts", 12.0);

  // Tunable numbers for manual testing
  private static final LoggedTunableNumber pivotKP =
      new LoggedTunableNumber("Intake/PivotKP", IntakeConstants.Tuneable_pivotKP);
  private static final LoggedTunableNumber pivotKI =
      new LoggedTunableNumber("Intake/PivotKI", IntakeConstants.Tuneable_pivotKI);
  private static final LoggedTunableNumber pivotKD =
      new LoggedTunableNumber("Intake/PivotKD", IntakeConstants.Tuneable_pivotKD);
  private static final LoggedTunableNumber pivotKG =
      new LoggedTunableNumber("Intake/PivotKG", IntakeConstants.Tuneable_pivotKG);
  private static final LoggedTunableNumber pivotKS =
      new LoggedTunableNumber("Intake/PivotKS", IntakeConstants.Tuneable_pivotKS);
  private static final LoggedTunableNumber pivotVelo =
      new LoggedTunableNumber("Intake/PivotVelo", IntakeConstants.Tuneable_pivot_ACCEL);
  private static final LoggedTunableNumber pivotAccel =
      new LoggedTunableNumber("Intake/PivotAccel", IntakeConstants.Tuneable_pivot_VELOCITY);
  private static final LoggedTunableNumber pivotJerk =
      new LoggedTunableNumber("Intake/PivotJerk", IntakeConstants.Tuneable_pivotJERK);

  private static final LoggedTunableNumber pivotManualTestVolts =
      new LoggedTunableNumber("Intake/PivotManualTestVolts", 1.0);

  private IntakeState currentState = IntakeState.NONE;

  public Intake(IntakeIO io, Feeder feeder) {
    this.io = io;
    this.feeder = feeder;
  }

  @Override
  public void periodic() {

    double timestamp = RobotTime.getTimestampSeconds();
    io.updateInputs(inputs);

    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/JamDetected", checkForJam());
    Logger.recordOutput("Intake/CurrentState", currentState);

    CoralStateTracker.updateIntake(isCoralInIntake());

    // Update PID/FF values if they have changed
    if (pivotKP.hasChanged(hashCode())
        || pivotKI.hasChanged(hashCode())
        || pivotKD.hasChanged(hashCode())
        || pivotKG.hasChanged(hashCode())
        || pivotKS.hasChanged(hashCode())
        || pivotVelo.hasChanged(hashCode())
        || pivotAccel.hasChanged(hashCode())
        || pivotJerk.hasChanged(hashCode())) {
      io.updatePivotPIDFF(
          pivotKP.get(),
          pivotKI.get(),
          pivotKD.get(),
          pivotKG.get(),
          pivotKS.get(),
          pivotVelo.get(),
          pivotAccel.get(),
          pivotJerk.get());
      Logger.recordOutput(
          "Intake/currentCommand",
          (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
    }

    Logger.recordOutput(
        getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
  }

  public void setRollerVoltage(double voltage) {
    io.setRollerVoltage(voltage);
  }

  public boolean isPivotAtSetpoint(double setpoint) {
    return Math.abs(inputs.pivotData.positionRotation() - setpoint)
        < IntakeConstants.PIVOT_TOLERANCE_ROTATIONS;
  }

  public double getCurrentPivotPosition() {
    return inputs.pivotData.positionRotation();
  }

  public boolean isCoralInIntake() {
    return inputs.canRangeData.tripped() && inputs.canRangeData.isSensorConnected();
  }

  private boolean checkForJam() {
    // return false;
    return io.checkRollerStalled() || feeder.checkForJam();
  }

  public double getRollerVelocityRPS() {
    return inputs.rollerData.velocityRPS();
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
            case NONE:
              break;
            case STOW:
              break;
            case INTAKE:
              // Check if coral is detected in feeder and automatically transition to IDLE
              if (CoralStateTracker.getCurrentPosition() == CoralPosition.AT_BACK_FEEDER
                  && RobotState.getSuperstructureState().isHandoffState()) {
                this.currentState = IntakeState.HAND_OFF;
              } else if (CoralStateTracker.getCurrentPosition() == CoralPosition.AT_BACK_FEEDER
                  || CoralStateTracker.getCurrentPosition()
                      == CoralPosition.STAGED_IN_END_EFFECTOR) {
                this.currentState = IntakeState.IDLE;
              }

              break;
            case REJECT_CORAL:
              break;
            case HAND_OFF:
              if (CoralStateTracker.getCurrentPosition() == CoralPosition.STAGED_IN_END_EFFECTOR
                  || CoralStateTracker.getCurrentPosition()
                      == CoralPosition.AT_SECOND_END_EFFECTOR) {
                this.currentState = IntakeState.IDLE;
              }
              break;
            case REJECT_INTAKE_CORAL:
              if (CoralStateTracker.getCurrentPosition() == CoralPosition.AT_BACK_FEEDER
                  && RobotState.getSuperstructureState().isHandoffState()) {
                this.currentState = IntakeState.REJECT_INTAKE_CORAL_HANDOFF;
              } else if (CoralStateTracker.getCurrentPosition() == CoralPosition.AT_BACK_FEEDER
                  || CoralStateTracker.getCurrentPosition()
                      == CoralPosition.STAGED_IN_END_EFFECTOR) {
                this.currentState = IntakeState.REJECT_INTAKE_CORAL_STAGED;
              }
              break;
            case REJECT_INTAKE_CORAL_STAGED:
              if ((CoralStateTracker.getCurrentPosition() == CoralPosition.AT_BACK_FEEDER
                      || CoralStateTracker.getCurrentPosition() == CoralPosition.AT_FRONT_FEEDER)
                  && RobotState.getSuperstructureState().isHandoffState()) {
                this.currentState = IntakeState.REJECT_INTAKE_CORAL_HANDOFF;
              }

            case REJECT_INTAKE_CORAL_HANDOFF:
              if (CoralStateTracker.getCurrentPosition() == CoralPosition.STAGED_IN_END_EFFECTOR
                  || CoralStateTracker.getCurrentPosition()
                      == CoralPosition.AT_SECOND_END_EFFECTOR) {
                this.currentState = IntakeState.REJECT_CORAL;
              }
              break;
            case SCORING:
              break;
            case SCORING_PREP:
              break;
            case IDLE:
              if ((CoralStateTracker.getCurrentPosition() == CoralPosition.AT_BACK_FEEDER
                      || CoralStateTracker.getCurrentPosition() == CoralPosition.AT_FRONT_FEEDER)
                  && RobotState.getSuperstructureState().isHandoffState()) {
                this.currentState = IntakeState.HAND_OFF;
              }
              break;
            default:
              this.currentState = IntakeState.IDLE;
              break;
          }

          // Execute motor commands based on current state
          switch (this.currentState) {
            case NONE:
              break;
            case STOW:
              this.io.setPivotPosition(IntakeConstants.PIVOT_UP_POSITION);
              this.io.setRollerVoltage(0);
              feeder.setRollerVoltage(FeederConstants.FEEDER_STOP_VOLTS);
              break;
            case INTAKE:
              this.io.setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION);
              this.io.setRollerVoltage(rollerIntakeVolts.get());
              feeder.setRollerVoltage(FeederConstants.FEEDER_IN_VOLTS);
              break;
            case REJECT_CORAL:
              this.io.setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION);
              this.io.setRollerVoltage(-rollerIntakeVolts.get());
              feeder.setRollerVoltage(FeederConstants.FEEDER_OUT_VOLTS);
              break;
            case REJECT_INTAKE_CORAL:
              this.io.setRollerVoltage(-rollerIntakeVolts.get());
              break;
            case REJECT_INTAKE_CORAL_STAGED:
              this.io.setRollerVoltage(-rollerIntakeVolts.get());
              feeder.setRollerVoltage(0);
              break;
            case REJECT_INTAKE_CORAL_HANDOFF:
              this.io.setRollerVoltage(-rollerIntakeVolts.get());
              feeder.setRollerVoltage(FeederConstants.FEEDER_IN_VOLTS);
              break;
            case HAND_OFF:
              this.io.setRollerVoltage(0);
              feeder.setRollerVoltage(FeederConstants.FEEDER_IN_VOLTS);
              break;
            case IDLE:
            default:
              this.io.setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION);
              this.io.setRollerVoltage(0);
              feeder.setRollerVoltage(FeederConstants.FEEDER_STOP_VOLTS);
              break;
          }
        },
        this,
        feeder);
  }

  public Command setIntakeStateCommand(IntakeState state) {
    return Commands.runOnce(() -> this.currentState = state, this);
  }

  public Command setPivotDown() {
    return Commands.runOnce(
        () -> this.io.setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION), this);
  }

  public Command setPivotUp() {
    return Commands.runOnce(
        () -> this.io.setPivotPosition(IntakeConstants.PIVOT_UP_POSITION), this);
  }

  public Command setPivotScoring() {
    return Commands.runOnce(
        () -> this.io.setPivotPosition(IntakeConstants.PIVOT_SCORING_POSITION), this);
  }

  public Command zeroPivotAtPivotUp() {
    return Commands.runOnce(() -> this.io.setPivotZero(), this);
  }

  public Command rejectCoral() {
    return Commands.run(() -> this.io.setRollerVoltage(-rollerRejectVolts.get()), this);
  }

  // Manual test functions for intake pivot
  public Command pivotManualTestForward() {
    return Commands.run(() -> this.io.setPivotVoltage(pivotManualTestVolts.get()), this);
  }

  public Command pivotManualTestReverse() {
    return Commands.run(() -> this.io.setPivotVoltage(-pivotManualTestVolts.get()), this);
  }

  public Command pivotStop() {
    return Commands.runOnce(() -> this.io.setPivotVoltage(0.0), this);
  }

  public Trigger intakeJamTrigger =
      new Trigger(() -> checkForJam()).debounce(IntakeConstants.DEJAM_DEBOUNCE_SECONDS);

  public Command dejamFeeder() {
    return Commands.sequence(
        Commands.runOnce(() -> feeder.setRollerVoltage(-feederVolts.getAsDouble())),
        Commands.waitSeconds(FeederConstants.DEJAM_DURATION_SECONDS),
        Commands.runOnce(() -> feeder.setRollerVoltage(0.0)));
  }

  public Command feederFWD() {
    return Commands.runOnce(() -> feeder.setRollerVoltage(feederVolts.getAsDouble()));
  }

  public Command feederRVS() {
    return Commands.runOnce(() -> feeder.setRollerVoltage(-feederVolts.getAsDouble()));
  }

  public Command feederSTOP() {
    return Commands.runOnce(() -> feeder.setRollerVoltage(0));
  }

  public Trigger rejectCoralIntakeTrigger =
      new Trigger(
              () -> {
                switch (CoralStateTracker.getCurrentPosition()) {
                  case AT_FRONT_FEEDER, AT_BACK_FEEDER:
                    return isCoralInIntake();

                  default:
                    return false;
                }
              })
          .debounce(0.1);

  public Trigger rejectCoralIntakeAndFeederTrigger =
      new Trigger(
              () -> {
                switch (CoralStateTracker.getCurrentPosition()) {
                  case STAGED_IN_END_EFFECTOR, AT_FIRST_END_EFFECTOR, AT_SECOND_END_EFFECTOR:
                    return isCoralInIntake();

                  default:
                    return false;
                }
              })
          .debounce(0.1);
}
