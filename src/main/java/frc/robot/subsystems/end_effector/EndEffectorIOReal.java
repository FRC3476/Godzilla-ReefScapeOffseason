package frc.robot.subsystems.end_effector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.util.PhoenixUtil;

public class EndEffectorIOReal implements EndEffectorIO {

  private TalonFX pivotTalonFX;
  private TalonFX rollerTalonFX;
  private CANrange coralCANRange;

  private MotionMagicVoltage pivot_m_request =
      new MotionMagicVoltage(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);
  private VelocityVoltage roller_m_request = new VelocityVoltage(0).withEnableFOC(true);

  // =====Logged Values=====
  StatusSignal<Angle> pivotPosition;
  StatusSignal<Voltage> pivotAppliedVolts;
  StatusSignal<Current> pivotTorqueCurrentAmps;
  StatusSignal<Current> pivotSupplyCurrentAmps;
  StatusSignal<Temperature> pivotTempCelsius;

  StatusSignal<Angle> rollerPosition;
  StatusSignal<Voltage> rollerAppliedVolts;
  StatusSignal<Current> rollerTorqueCurrentAmps;
  StatusSignal<Current> rollerSupplyCurrentAmps;
  StatusSignal<Temperature> rollerTempCelsius;

  StatusSignal<Boolean> rangeIsTripped;

  public EndEffectorIOReal() {
    pivotTalonFX = new TalonFX(EndEffectorConstants.pivotID);
    rollerTalonFX = new TalonFX(EndEffectorConstants.rollerID);
    coralCANRange = new CANrange(EndEffectorConstants.coralCANRangeID);

    pivotTalonFX.getConfigurator().apply(EndEffectorConstants.PIVOT_TALON_CONFIG);
    rollerTalonFX.getConfigurator().apply(EndEffectorConstants.ROLLER_TALON_CONFIG);

    pivotPosition = pivotTalonFX.getPosition();
    pivotAppliedVolts = pivotTalonFX.getMotorVoltage();
    pivotTorqueCurrentAmps = pivotTalonFX.getTorqueCurrent();
    pivotSupplyCurrentAmps = pivotTalonFX.getSupplyCurrent();
    pivotTempCelsius = pivotTalonFX.getDeviceTemp();

    rollerPosition = rollerTalonFX.getPosition();
    rollerAppliedVolts = rollerTalonFX.getMotorVoltage();
    rollerTorqueCurrentAmps = rollerTalonFX.getTorqueCurrent();
    rollerSupplyCurrentAmps = rollerTalonFX.getSupplyCurrent();
    rollerTempCelsius = rollerTalonFX.getDeviceTemp();

    rangeIsTripped = coralCANRange.getIsDetected();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotAppliedVolts,
        pivotTorqueCurrentAmps,
        pivotSupplyCurrentAmps,
        pivotTempCelsius,
        rollerPosition,
        rollerAppliedVolts,
        rollerTorqueCurrentAmps,
        rollerSupplyCurrentAmps,
        rollerTempCelsius,
        rangeIsTripped);
    ParentDevice.optimizeBusUtilizationForAll(pivotTalonFX, rollerTalonFX, coralCANRange);
    PhoenixUtil.registerSignals(
        true,
        pivotPosition,
        pivotAppliedVolts,
        pivotTorqueCurrentAmps,
        pivotSupplyCurrentAmps,
        pivotTempCelsius,
        rollerPosition,
        rollerAppliedVolts,
        rollerTorqueCurrentAmps,
        rollerSupplyCurrentAmps,
        rollerTempCelsius,
        rangeIsTripped);
  }

  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.data =
        new EndEffectorIOData(
            BaseStatusSignal.isAllGood(
                pivotPosition,
                pivotAppliedVolts,
                pivotTorqueCurrentAmps,
                pivotSupplyCurrentAmps,
                pivotTempCelsius),
            BaseStatusSignal.isAllGood(
                rollerPosition,
                rollerAppliedVolts,
                rollerTorqueCurrentAmps,
                rollerSupplyCurrentAmps,
                rollerTempCelsius),
            BaseStatusSignal.isAllGood(rangeIsTripped),
            Units.rotationsToRadians(pivotPosition.getValueAsDouble()),
            pivotAppliedVolts.getValueAsDouble(),
            pivotTorqueCurrentAmps.getValueAsDouble(),
            pivotSupplyCurrentAmps.getValueAsDouble(),
            pivotTempCelsius.getValueAsDouble(),
            Units.rotationsToRadians(rollerPosition.getValueAsDouble()),
            rollerAppliedVolts.getValueAsDouble(),
            rollerTorqueCurrentAmps.getValueAsDouble(),
            rollerSupplyCurrentAmps.getValueAsDouble(),
            rollerTempCelsius.getValueAsDouble(),
            rangeIsTripped.getValue());
  }

  public void setRollerVelocity(double velocity) {
    rollerTalonFX.setControl(roller_m_request.withVelocity(velocity));
  }

  public void setRollerVoltage(double voltage) {
    rollerTalonFX.setControl(new VoltageOut(voltage));
  }
}
