package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Intake/RollerVolts", 12.0);
  private static final LoggedTunableNumber rollerRejectVolts =
      new LoggedTunableNumber("Intake/RollerRejectVolts", 12.0); // Placeholder value

  private static Intake intakeSubsystem;

  public static Intake getInstance() {
    if (intakeSubsystem == null) {
      intakeSubsystem = new Intake(new IntakeIOReal());
    }
    return intakeSubsystem;
  }

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean isPivotAtSetpoint(double setpoint) {
    return Math.abs(inputs.pivotData.positionRad() - setpoint)
        < frc.robot.Constants.IntakeConstants.PIVOT_TOLERANCE_RAD;
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

  public Command rejectCoral() {
    return Commands.run(() -> this.io.setRollerVoltage(-rollerRejectVolts.get()), this);
  }

  public Command intakePivotStow() {
    return Commands.runOnce(
        () ->
            this.io.setPivotPosition(
                frc.robot.Constants.IntakeConstants.INTAKE_PIVOT_STOWED_POSITION),
        this);
  }

  public Command engageCoralL1() {
    return Commands.runOnce(
        () ->
            this.io.setLvl1BlockerPosition(
                Constants.IntakeConstants.L1_BLOCKER_CORAL_ENGAGED_POSITION),
        this);
  }

  public Command disengageCoralL1() {
    return Commands.runOnce(
        () ->
            this.io.setLvl1BlockerPosition(
                Constants.IntakeConstants.L1_BLOCKER_CORAL_DISENGAGED_POSITION),
        this);
  }

  public Command scoreIntakeL1() {
    return Commands.sequence(
        Commands.runOnce(
            () -> io.setPivotPosition(frc.robot.Constants.IntakeConstants.PIVOT_L1_SETPOINT_RAD),
            this),
        Commands.waitUntil(
            () -> isPivotAtSetpoint(frc.robot.Constants.IntakeConstants.PIVOT_L1_SETPOINT_RAD)),
        Commands.runOnce(
            () -> io.setRollerVoltage(frc.robot.Constants.IntakeConstants.ROLLER_L1_SETPOINT_VOLTS),
            this));
  }
}
