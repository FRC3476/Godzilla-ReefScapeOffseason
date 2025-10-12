package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.MotorStallDetection;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {
  protected TalonFX pivotMotor;
  protected TalonFX rollerMotor;

  protected CANcoder canCoder;
  protected CANrange canRange;

  private final VoltageOut pivotVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut rollerVoltageRequest = new VoltageOut(0.0);
  private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0.0);

  // Pivot motor status signals
  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Current> pivotSupplyCurrent;
  private final StatusSignal<Current> pivotStatorCurrent;
  private final StatusSignal<Temperature> pivotTemperature;
  private final StatusSignal<AngularVelocity> pivotVelocityRPS;
  private final StatusSignal<Angle> pivotPositionRot;
  private final StatusSignal<Double> pivotPositionSetpointRotations;

  // Roller motor status signals
  private final StatusSignal<Voltage> rollerVoltage;
  private final StatusSignal<Current> rollerSupplyCurrent;
  private final StatusSignal<Current> rollerStatorCurrent;
  private final StatusSignal<Temperature> rollerTemperature;
  private final StatusSignal<AngularVelocity> rollerVelocityRPS;

  // CANCoder status signals
  private final StatusSignal<Angle> canCoderPositionRotations;
  private final StatusSignal<AngularVelocity> canCoderVelocityRPS;

  // CANRange status signals
  private final StatusSignal<Boolean> canRangeTripped;
  private final StatusSignal<Double> canRangeSignalStrength;
  private final StatusSignal<Distance> canRangeDistance;

  private final BaseStatusSignal[] signals;

  public IntakeIOReal() {
    // Initialize hardware
    pivotMotor = new TalonFX(IntakeConstants.intakePivotID, Constants.MISC_CANIVORE);
    rollerMotor = new TalonFX(IntakeConstants.intakeRollerID, Constants.MISC_CANIVORE);
    canCoder = new CANcoder(IntakeConstants.CANCODER_ID, Constants.MISC_CANIVORE);
    canRange = new CANrange(IntakeConstants.CANRANGE_ID, Constants.MISC_CANIVORE);

    // Configure pivot motor
    PhoenixUtil.tryUntilOk(
        5, () -> pivotMotor.getConfigurator().apply(IntakeConstants.PIVOT_TALON_CONFIG));

    // Configure roller motor
    PhoenixUtil.tryUntilOk(
        5, () -> rollerMotor.getConfigurator().apply(IntakeConstants.ROLLER_TALON_CONFIG));

    PhoenixUtil.tryUntilOk(
        5, () -> canRange.getConfigurator().apply(IntakeConstants.CANRANGE_CONFIG));

    // Configure CANRange
    var canRangeConfig = new CANrangeConfiguration();
    canRangeConfig.ProximityParams.ProximityThreshold = 0.05; // 5cm detection threshold
    canRangeConfig.ProximityParams.ProximityHysteresis = 0.01; // 1cm hysteresis
    PhoenixUtil.tryUntilOk(5, () -> canRange.getConfigurator().apply(canRangeConfig));

    // Initialize status signals
    pivotVoltage = pivotMotor.getMotorVoltage();
    pivotSupplyCurrent = pivotMotor.getSupplyCurrent();
    pivotStatorCurrent = pivotMotor.getStatorCurrent();
    pivotTemperature = pivotMotor.getDeviceTemp();
    pivotVelocityRPS = pivotMotor.getVelocity();
    pivotPositionRot = pivotMotor.getPosition();
    pivotPositionSetpointRotations = pivotMotor.getClosedLoopReference();

    rollerVoltage = rollerMotor.getMotorVoltage();
    rollerSupplyCurrent = rollerMotor.getSupplyCurrent();
    rollerStatorCurrent = rollerMotor.getStatorCurrent();
    rollerTemperature = rollerMotor.getDeviceTemp();
    rollerVelocityRPS = rollerMotor.getRotorVelocity();

    canCoderPositionRotations = canCoder.getAbsolutePosition();
    canCoderVelocityRPS = canCoder.getVelocity();

    canRangeTripped = canRange.getIsDetected();
    canRangeSignalStrength = canRange.getSignalStrength();
    canRangeDistance = canRange.getDistance();

    signals =
        new BaseStatusSignal[] {
          pivotVoltage,
          pivotSupplyCurrent,
          pivotStatorCurrent,
          pivotTemperature,
          pivotVelocityRPS,
          pivotPositionRot,
          pivotPositionSetpointRotations,
          rollerVoltage,
          rollerSupplyCurrent,
          rollerStatorCurrent,
          rollerTemperature,
          rollerVelocityRPS,
          canCoderPositionRotations,
          canCoderVelocityRPS,
          canRangeTripped,
          canRangeSignalStrength,
          canRangeDistance
        };

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotVoltage,
        pivotSupplyCurrent,
        pivotStatorCurrent,
        pivotTemperature,
        pivotVelocityRPS,
        pivotPositionRot,
        pivotPositionSetpointRotations,
        rollerVoltage,
        rollerSupplyCurrent,
        rollerStatorCurrent,
        rollerTemperature,
        rollerVelocityRPS,
        canCoderPositionRotations,
        canCoderVelocityRPS,
        canRangeTripped,
        canRangeSignalStrength,
        canRangeDistance);

    pivotMotor.optimizeBusUtilization();
    rollerMotor.optimizeBusUtilization();
    canCoder.optimizeBusUtilization();
    canRange.optimizeBusUtilization();

    setPositionFromAbsolute();
  }

  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(signals);

    inputs.pivotData =
        new PivotData(
            BaseStatusSignal.isAllGood(
                pivotVoltage,
                pivotSupplyCurrent,
                pivotStatorCurrent,
                pivotTemperature,
                pivotVelocityRPS,
                pivotPositionRot),
            pivotVoltage.getValueAsDouble(),
            pivotSupplyCurrent.getValueAsDouble(),
            pivotStatorCurrent.getValueAsDouble(),
            pivotTemperature.getValueAsDouble(),
            pivotVelocityRPS.getValueAsDouble(),
            pivotPositionRot.getValueAsDouble(),
            pivotPositionSetpointRotations.getValueAsDouble());

    inputs.rollerData =
        new RollerData(
            BaseStatusSignal.isAllGood(
                rollerVoltage,
                rollerSupplyCurrent,
                rollerStatorCurrent,
                rollerTemperature,
                rollerVelocityRPS),
            rollerVoltage.getValueAsDouble(),
            rollerSupplyCurrent.getValueAsDouble(),
            rollerStatorCurrent.getValueAsDouble(),
            rollerTemperature.getValueAsDouble(),
            rollerVelocityRPS.getValueAsDouble());

    inputs.canCoderData =
        new CanCoderData(
            BaseStatusSignal.isAllGood(canCoderPositionRotations, canCoderVelocityRPS),
            canCoderPositionRotations.getValueAsDouble(),
            canCoderVelocityRPS.getValueAsDouble());

    inputs.canRangeData =
        new CanRangeData(
            BaseStatusSignal.isAllGood(canRangeTripped, canRangeDistance, canRangeSignalStrength),
            canRangeTripped.getValue(),
            canRangeSignalStrength.getValueAsDouble(),
            canRangeDistance.getValueAsDouble());
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotMotor.setControl(pivotVoltageRequest.withOutput(voltage));
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollerMotor.setControl(rollerVoltageRequest.withOutput(voltage));
  }

  @Override
  public void setPivotPosition(double positionRotations) {
    pivotMotor.setControl(pivotPositionRequest.withPosition(positionRotations));
  }

  @Override
  public void updatePivotPIDFF(
      double kP,
      double kI,
      double kD,
      double kG,
      double kS,
      double velo,
      double accel,
      double jerk) {
    var pivotConfig = new TalonFXConfiguration();
    pivotMotor.getConfigurator().refresh(pivotConfig);
    pivotConfig.Slot0.kP = kP;
    pivotConfig.Slot0.kI = kI;
    pivotConfig.Slot0.kD = kD;
    pivotConfig.Slot0.kG = kG;
    pivotConfig.Slot0.kS = kS;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = velo;
    pivotConfig.MotionMagic.MotionMagicAcceleration = accel;
    pivotConfig.MotionMagic.MotionMagicJerk = jerk;
    PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.050));
  }

  @Override
  public boolean checkRollerStalled() {
    return MotorStallDetection.isMotorStalled(
        rollerStatorCurrent.getValueAsDouble(),
        rollerVelocityRPS.getValueAsDouble(),
        IntakeConstants.ROLLER_STALLED_CURRENT_A,
        IntakeConstants.ROLLER_STALLED_RPS);
  }

  @Override
  public void setPivotZero() {
    // Configure CANCoder
    PhoenixUtil.tryUntilOk(
        5, () -> canCoder.getConfigurator().apply(IntakeConstants.CANCODER_CONFIG));

    canCoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(0));

    Util.sleep(Constants.PIVOT_ZERO_SLEEP_MS);
    Logger.recordOutput(
        "Intake/absolutePostionBeforeOffset", canCoder.getAbsolutePosition().getValueAsDouble());

    double intakeUpAbsoluteRotations =
        IntakeConstants.PIVOT_UP_POSITION * IntakeConstants.PIVOT_STM;
    double magnetOffset =
        intakeUpAbsoluteRotations - canCoder.getAbsolutePosition().getValueAsDouble();
    magnetOffset = Util.rangeModulo(magnetOffset, 0.5, -0.5);

    canCoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(magnetOffset));
    Util.sleep(Constants.PIVOT_ZERO_SLEEP_MS);
    Logger.recordOutput(
        "Intake/absolutePostionAfterOffset", canCoder.getAbsolutePosition().getValueAsDouble());
    setPositionFromAbsolute();
  }

  private void setPositionFromAbsolute() {
    // Need to do this because the canCoder wraps from its 0 position.
    canCoder.setPosition(
        canCoder.getAbsolutePosition().getValueAsDouble()
            + Math.round(IntakeConstants.PIVOT_UP_POSITION * IntakeConstants.PIVOT_STM));
  }
}
