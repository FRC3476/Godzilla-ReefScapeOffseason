package frc.robot.subsystems.end_effector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.util.CANDiagnostics;
import frc.robot.util.MotorStallDetection;
import frc.robot.util.PhoenixUtil;

public class ClawIOReal implements ClawIO {

  protected TalonFX rollerTalonFX;
  private CANrange firstCoralCANRange;
  private CANrange secondCoralCANRange;

  private TorqueCurrentFOC roller_c_request =
      new TorqueCurrentFOC(EndEffectorConstants.CLAW_HOLD_ALGAE_AMPS);
  private VoltageOut roller_m_request = new VoltageOut(0).withEnableFOC(true);

  StatusSignal<AngularVelocity> rollerVelocityRPS;
  StatusSignal<Voltage> rollerAppliedVolts;
  StatusSignal<Current> rollerStatorCurrentAmps;
  StatusSignal<Current> rollerSupplyCurrentAmps;
  StatusSignal<Temperature> rollerTempCelsius;

  StatusSignal<Boolean> firstRangeIsTripped;
  StatusSignal<Boolean> secondRangeIsTripped;

  private final BaseStatusSignal[] signals;

  public ClawIOReal() {
    rollerTalonFX = new TalonFX(EndEffectorConstants.rollerID, Constants.MISC_CANIVORE);
    firstCoralCANRange =
        new CANrange(EndEffectorConstants.FIRST_CORAL_CANRANGE_ID, Constants.MISC_CANIVORE);
    secondCoralCANRange =
        new CANrange(EndEffectorConstants.SECOND_CORAL_CANRANGE_ID, Constants.MISC_CANIVORE);
    PhoenixUtil.tryUntilOk(
        5, () -> firstCoralCANRange.getConfigurator().apply(EndEffectorConstants.CANRANGE_CONFIG));
    PhoenixUtil.tryUntilOk(
        5, () -> secondCoralCANRange.getConfigurator().apply(EndEffectorConstants.CANRANGE_CONFIG));
    PhoenixUtil.tryUntilOk(
        5, () -> rollerTalonFX.getConfigurator().apply(EndEffectorConstants.ROLLER_TALON_CONFIG));

    rollerVelocityRPS = rollerTalonFX.getRotorVelocity();
    rollerAppliedVolts = rollerTalonFX.getMotorVoltage();
    rollerStatorCurrentAmps = rollerTalonFX.getStatorCurrent();
    rollerSupplyCurrentAmps = rollerTalonFX.getSupplyCurrent();
    rollerTempCelsius = rollerTalonFX.getDeviceTemp();

    firstRangeIsTripped = firstCoralCANRange.getIsDetected();
    secondRangeIsTripped = secondCoralCANRange.getIsDetected();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rollerVelocityRPS,
        rollerAppliedVolts,
        rollerStatorCurrentAmps,
        rollerSupplyCurrentAmps,
        rollerTempCelsius,
        firstRangeIsTripped,
        secondRangeIsTripped);
    ParentDevice.optimizeBusUtilizationForAll(
        rollerTalonFX, firstCoralCANRange, secondCoralCANRange);

    signals =
        new BaseStatusSignal[] {
          rollerVelocityRPS,
          rollerAppliedVolts,
          rollerStatorCurrentAmps,
          rollerSupplyCurrentAmps,
          rollerTempCelsius,
          firstRangeIsTripped,
          secondRangeIsTripped
        };
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    BaseStatusSignal.refreshAll(signals);
    CANDiagnostics.checkSignalHealth("Claw", signals);

    inputs.rollerData =
        new EE_RollerData(
            BaseStatusSignal.isAllGood(
                rollerVelocityRPS,
                rollerAppliedVolts,
                rollerStatorCurrentAmps,
                rollerSupplyCurrentAmps,
                rollerTempCelsius),
            rollerVelocityRPS.getValueAsDouble(),
            rollerAppliedVolts.getValueAsDouble(),
            rollerStatorCurrentAmps.getValueAsDouble(),
            rollerSupplyCurrentAmps.getValueAsDouble(),
            rollerTempCelsius.getValueAsDouble());
    inputs.firstCANRangeData =
        new EE_CANRangeData(
            BaseStatusSignal.isAllGood(firstRangeIsTripped), firstRangeIsTripped.getValue());
    inputs.secondCANRangeData =
        new EE_CANRangeData(
            BaseStatusSignal.isAllGood(secondRangeIsTripped), secondRangeIsTripped.getValue());
  }

  public void setTorqueCurrent(double amps) {
    rollerTalonFX.setControl(roller_c_request.withOutput(amps));
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollerTalonFX.setControl(roller_m_request.withOutput(voltage));
  }

  @Override
  public boolean checkRollerStalled() {
    return MotorStallDetection.isMotorStalled(
        rollerStatorCurrentAmps.getValueAsDouble(),
        rollerVelocityRPS.getValueAsDouble(),
        EndEffectorConstants.ROLLER_STALLED_CURRENT,
        EndEffectorConstants.ROLLER_STALLED_RPS);
  }
}
