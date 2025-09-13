package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.IntakeState;
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
    return Commands.run(() -> this.io.setRollerVoltage(rollerIntakeVolts.get()), this);
  }

  public Command intakeRVS() {
    return Commands.run(() -> this.io.setRollerVoltage(-rollerIntakeVolts.get()), this);
  }

  public Command intakeSTOP() {
    return Commands.run(() -> this.io.setRollerVoltage(0), this);
  }

  public Command intakeDefault() {
    switch (this.currentState) {
      case STOW:
        return Commands.run(() -> {
          this.io.setPivotPosition(frc.robot.Constants.IntakeConstants.PIVOT_UP_POSITION);
          this.io.setRollerVoltage(0);
        }, this);
      case INTAKE_L1:
        return Commands.run(() -> {
          this.io.setPivotPosition(frc.robot.Constants.IntakeConstants.PIVOT_INTAKE_POSITION);
          this.io.setRollerVoltage(rollerIntakeVolts.get());
          this.io.setLvl1BlockerPosition(frc.robot.Constants.IntakeConstants.L1_BLOCKER_ENGAGED_POSITION);
        }, this);
      case INTAKE:
        return Commands.sequence(
          Commands.run(() -> {
            this.io.setPivotPosition(frc.robot.Constants.IntakeConstants.PIVOT_INTAKE_POSITION);
            this.io.setRollerVoltage(rollerIntakeVolts.get());
            this.io.setLvl1BlockerPosition(frc.robot.Constants.IntakeConstants.L1_BLOCKER_DISENGAGED_POSITION);
          }, this),
          Commands.waitUntil(this::isCoralInIntake),
          intakeSTOP()
        );
      case REJECT_CORAL:
        return Commands.run(() -> {
          this.io.setPivotPosition(frc.robot.Constants.IntakeConstants.PIVOT_INTAKE_POSITION);
          this.io.setRollerVoltage(-rollerIntakeVolts.get());
        }, this);
      case IDLE:
        return Commands.run(() -> {
          this.io.setPivotPosition(frc.robot.Constants.IntakeConstants.PIVOT_INTAKE_POSITION);
          this.io.setRollerVoltage(0);
        }, this);
      case HAND_OFF:
        return Commands.run(() -> this.io.setRollerVoltage(0), this);
      case SCORING:
        return Commands.sequence(
          Commands.run(() -> {
            this.io.setPivotPosition(frc.robot.Constants.IntakeConstants.PIVOT_SCORING_POSITION);
            this.io.setRollerVoltage(frc.robot.Constants.IntakeConstants.ROLLER_SCORING_OUT_VOLTS);
            this.io.setLvl1BlockerPosition(frc.robot.Constants.IntakeConstants.L1_BLOCKER_ENGAGED_POSITION);
          }, this)
        );
      default:
        return Commands.none();
    }
  }

    public Command setIntakeState(IntakeState state) {
      return Commands.runOnce(() -> this.currentState = state, this);
    }

    public Command movePivotDown() {
      return Commands.run(() -> this.io.setPivotPosition(frc.robot.Constants.IntakeConstants.PIVOT_INTAKE_POSITION), this);
  }
}
