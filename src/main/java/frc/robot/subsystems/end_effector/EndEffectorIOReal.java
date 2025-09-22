package frc.robot.subsystems.end_effector;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.util.MotorStallDetection;
import frc.robot.util.PhoenixUtil;
import org.dyn4j.geometry.Rotation;

public class EndEffectorIOReal implements EndEffectorIO {

  private TalonFX pivotTalonFX;
  private TalonFX rollerTalonFX;
  private CANrange coralCANRange;

  private MotionMagicVoltage pivot_m_request =
      new MotionMagicVoltage(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);

  private VoltageOut roller_m_request =
      new VoltageOut(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);

  // =====Logged Values=====
  StatusSignal<Angle> pivotPosition;
  StatusSignal<Voltage> pivotAppliedVolts;
  StatusSignal<Current> pivotTorqueCurrentAmps;
  StatusSignal<Current> pivotSupplyCurrentAmps;
  StatusSignal<Temperature> pivotTempCelsius;

  StatusSignal<AngularVelocity> rollerVelocityRPS;
  StatusSignal<Voltage> rollerAppliedVolts;
  StatusSignal<Current> rollerTorqueCurrentAmps;
  StatusSignal<Current> rollerSupplyCurrentAmps;
  StatusSignal<Temperature> rollerTempCelsius;

  StatusSignal<Boolean> rangeIsTripped;

  public EndEffectorIOReal() {
    pivotTalonFX = new TalonFX(EndEffectorConstants.pivotID);
    rollerTalonFX = new TalonFX(EndEffectorConstants.rollerID);
    coralCANRange = new CANrange(EndEffectorConstants.FIRST_CORAL_CANRANGE_ID);

    PhoenixUtil.tryUntilOk(
        5, () -> pivotTalonFX.getConfigurator().apply(EndEffectorConstants.PIVOT_TALON_CONFIG));
    PhoenixUtil.tryUntilOk(
        5, () -> rollerTalonFX.getConfigurator().apply(EndEffectorConstants.ROLLER_TALON_CONFIG));

    pivotPosition = pivotTalonFX.getPosition();
    pivotAppliedVolts = pivotTalonFX.getMotorVoltage();
    pivotTorqueCurrentAmps = pivotTalonFX.getTorqueCurrent();
    pivotSupplyCurrentAmps = pivotTalonFX.getSupplyCurrent();
    pivotTempCelsius = pivotTalonFX.getDeviceTemp();

    rollerVelocityRPS = rollerTalonFX.getRotorVelocity();
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
        rollerVelocityRPS,
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
        rollerVelocityRPS,
        rollerAppliedVolts,
        rollerTorqueCurrentAmps,
        rollerSupplyCurrentAmps,
        rollerTempCelsius,
        rangeIsTripped);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.pivotData =
        new EE_PivotData(
            BaseStatusSignal.isAllGood(
                pivotPosition,
                pivotAppliedVolts,
                pivotTorqueCurrentAmps,
                pivotSupplyCurrentAmps,
                pivotTempCelsius),
            Units.rotationsToRadians(pivotPosition.getValueAsDouble()),
            pivotAppliedVolts.getValueAsDouble(),
            pivotTorqueCurrentAmps.getValueAsDouble(),
            pivotSupplyCurrentAmps.getValueAsDouble(),
            pivotTempCelsius.getValueAsDouble());
    inputs.rollerData =
        new EE_RollerData(
            BaseStatusSignal.isAllGood(
                rollerVelocityRPS,
                rollerAppliedVolts,
                rollerTorqueCurrentAmps,
                rollerSupplyCurrentAmps,
                rollerTempCelsius),
            rollerVelocityRPS.getValueAsDouble(),
            rollerAppliedVolts.getValueAsDouble(),
            rollerTorqueCurrentAmps.getValueAsDouble(),
            rollerSupplyCurrentAmps.getValueAsDouble(),
            rollerTempCelsius.getValueAsDouble());
    inputs.canRangeData =
        new EE_CANRangeData(BaseStatusSignal.isAllGood(rangeIsTripped), rangeIsTripped.getValue());
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollerTalonFX.setControl(roller_m_request.withOutput(voltage));
  }

  @Override
  public void setPivotVoltage(double voltage) {
    rollerTalonFX.setControl(pivot_m_request.withPosition(voltage));
  }

  @Override
  public void setPivotPosition(double position) {
    pivotTalonFX.setControl(pivot_m_request.withPosition(Rotation.convertFrom(position, Degree)));
  }

  @Override
  public boolean checkRollerStalled() {
    return MotorStallDetection.isMotorStalled(
        rollerTalonFX,
        EndEffectorConstants.ROLLER_STALLED_CURRENT,
        EndEffectorConstants.ROLLER_STALLED_RPS);
  }
}
