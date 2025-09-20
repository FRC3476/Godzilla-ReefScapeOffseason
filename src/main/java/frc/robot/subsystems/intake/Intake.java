package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Intake/RollerVolts", 12.0);

  private IntakeState currentState = IntakeState.IDLE;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean isCoralInIntake() {
    return inputs.canRangeData.tripped() && inputs.canRangeData.isSensorConnected();
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
    return Commands.run(() -> {
      switch (this.currentState) {
        case STOW:
          break;
        case INTAKE_L1:
          break;
        case INTAKE:
          // Check if coral is detected in feeder and automatically transition to IDLE
          if (Feeder.getInstance().isCoralInFeeder()) {
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
          Feeder.getInstance().setRollerVoltage(FeederConstants.FEEDER_STOP_VOLTS);
          break;
        case INTAKE_L1:
          this.io.setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION);
          this.io.setRollerVoltage(rollerIntakeVolts.get());
          this.io.setLvl1BlockerPosition(IntakeConstants.L1_BLOCKER_ENGAGED_POSITION);
          Feeder.getInstance().setRollerVoltage(FeederConstants.FEEDER_IN_VOLTS);
          break;
        case INTAKE:
          this.io.setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION);
          this.io.setRollerVoltage(rollerIntakeVolts.get());
          this.io.setLvl1BlockerPosition(IntakeConstants.L1_BLOCKER_DISENGAGED_POSITION);
          Feeder.getInstance().setRollerVoltage(FeederConstants.FEEDER_IN_VOLTS);
          break;
        case REJECT_CORAL:
          this.io.setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION);
          this.io.setRollerVoltage(-rollerIntakeVolts.get());
          Feeder.getInstance().setRollerVoltage(FeederConstants.FEEDER_OUT_VOLTS);
          break;
        case HAND_OFF:
          this.io.setRollerVoltage(0);
          Feeder.getInstance().setRollerVoltage(FeederConstants.FEEDER_IN_VOLTS);
          break;
        case SCORING:
          this.io.setPivotPosition(IntakeConstants.PIVOT_SCORING_POSITION);
          this.io.setRollerVoltage(IntakeConstants.ROLLER_SCORING_OUT_VOLTS);
          this.io.setLvl1BlockerPosition(IntakeConstants.L1_BLOCKER_ENGAGED_POSITION);
          Feeder.getInstance().setRollerVoltage(FeederConstants.FEEDER_STOP_VOLTS);
          break;
        case SCORING_PREP:
          this.io.setPivotPosition(IntakeConstants.SCORING_PREP_PIVOT_POSITION_RAD);
          this.io.setRollerVoltage(0);
          this.io.setLvl1BlockerPosition(IntakeConstants.L1_BLOCKER_ENGAGED_POSITION);
          Feeder.getInstance().setRollerVoltage(FeederConstants.FEEDER_STOP_VOLTS);
          break;
        case IDLE:
        default:
          this.io.setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION);
          this.io.setRollerVoltage(0);
          Feeder.getInstance().setRollerVoltage(FeederConstants.FEEDER_STOP_VOLTS);
          break;
      }
    }, this);
  }

    public Command setIntakeState(IntakeState state) {
      return Commands.runOnce(() -> this.currentState = state, this);
    }

    public Command movePivotDown() {
      return Commands.runOnce(() -> this.io.setPivotPosition(IntakeConstants.PIVOT_INTAKE_POSITION), this);
  }
}
