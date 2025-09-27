package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

public class IntakeIOReal implements IntakeIO {
  protected TalonFX pivotMotor;
  protected TalonFX rollerMotor;
  protected TalonFX lvl1blockerMotor;

  protected CANcoder canCoder;
  protected CANrange canRange;

  private final VoltageOut pivotVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut rollerVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut lvl1blockerVoltageRequest = new VoltageOut(0.0);
  private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0.0);
  private final MotionMagicVoltage lvl1blockerPositionRequest = new MotionMagicVoltage(0.0);

  // Pivot motor status signals
  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Current> pivotSupplyCurrent;
  private final StatusSignal<Current> pivotStatorCurrent;
  private final StatusSignal<Temperature> pivotTemperature;
  private final StatusSignal<AngularVelocity> pivotVelocityRPS;
  private final StatusSignal<Angle> pivotPositionRad;

  // Roller motor status signals
  private final StatusSignal<Voltage> rollerVoltage;
  private final StatusSignal<Current> rollerSupplyCurrent;
  private final StatusSignal<Current> rollerStatorCurrent;
  private final StatusSignal<Temperature> rollerTemperature;
  private final StatusSignal<AngularVelocity> rollerVelocityRPS;

  // Lvl1Blocker motor status signals
  private final StatusSignal<Voltage> lvl1blockerVoltage;
  private final StatusSignal<Current> lvl1blockerSupplyCurrent;
  private final StatusSignal<Current> lvl1blockerStatorCurrent;
  private final StatusSignal<Temperature> lvl1blockerTemperature;
  private final StatusSignal<AngularVelocity> lvl1blockerVelocityRPS;
  private final StatusSignal<Angle> lvl1blockerPositionRad;

  // CANCoder status signals
  private final StatusSignal<Angle> canCoderPositionRad;
  private final StatusSignal<AngularVelocity> canCoderVelocityRPS;

  // CANRange status signals
  private final StatusSignal<Boolean> canRangeTripped;
  private final StatusSignal<Double> canRangeSignalStrength;
  private final StatusSignal<Distance> canRangeDistance;

  private final BaseStatusSignal[] signals;

  public IntakeIOReal() {
    // Initialize hardware
    pivotMotor = new TalonFX(IntakeConstants.intakePivotID, Constants.misc_canivore);
    rollerMotor = new TalonFX(IntakeConstants.intakeRollerID, Constants.misc_canivore);
    lvl1blockerMotor = new TalonFX(IntakeConstants.intakelvl1BlockerID, Constants.misc_canivore);
    canCoder = new CANcoder(IntakeConstants.CANCODER_ID, Constants.misc_canivore);
    canRange = new CANrange(IntakeConstants.CANRANGE_ID, Constants.misc_canivore);

    // Configure pivot motor
    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.PIVOT_MAX_SUPPLY_CURRENT_LIMIT;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.PIVOT_MAX_STATOR_CURRENT_LIMIT;
    pivotConfig.Slot0.kP = IntakeConstants.pivotKP;
    pivotConfig.Slot0.kI = IntakeConstants.pivotKI;
    pivotConfig.Slot0.kD = IntakeConstants.pivotKD;
    pivotConfig.Slot0.kG = IntakeConstants.pivotKG;
    pivotConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.pivotMAX_ACCEL;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.pivotMAX_VELOCITY;
    pivotConfig.MotionMagic.MotionMagicJerk = IntakeConstants.pivotJERK;
    PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig));

    // Configure roller motor
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_MAX_SUPPLY_CURRENT_LIMIT;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_MAX_STATOR_CURRENT_LIMIT;
    PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerConfig));

    // Configure lvl1blocker motor
    var lvl1blockerConfig = new TalonFXConfiguration();
    lvl1blockerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    lvl1blockerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    lvl1blockerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    lvl1blockerConfig.CurrentLimits.SupplyCurrentLimit =
        IntakeConstants.L1_MAX_SUPPLY_CURRENT_LIMIT;
    lvl1blockerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    lvl1blockerConfig.CurrentLimits.StatorCurrentLimit =
        IntakeConstants.L1_MAX_STATOR_CURRENT_LIMIT;
    lvl1blockerConfig.Slot0.kP = IntakeConstants.lvl1blockerKP;
    lvl1blockerConfig.Slot0.kI = IntakeConstants.lvl1blockerKI;
    lvl1blockerConfig.Slot0.kD = IntakeConstants.lvl1blockerKD;
    lvl1blockerConfig.Slot0.kG = IntakeConstants.lvl1blockerKG;
    lvl1blockerConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.lvl1blockerMAX_ACCEL;
    lvl1blockerConfig.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConstants.lvl1blockerMAX_VELOCITY;
    lvl1blockerConfig.MotionMagic.MotionMagicJerk = IntakeConstants.lvl1blockerJERK;
    PhoenixUtil.tryUntilOk(5, () -> lvl1blockerMotor.getConfigurator().apply(lvl1blockerConfig));

    // Configure CANCoder
    var canCoderConfig = new CANcoderConfiguration();
    PhoenixUtil.tryUntilOk(5, () -> canCoder.getConfigurator().apply(canCoderConfig));

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
    pivotVelocityRPS = pivotMotor.getRotorVelocity();
    pivotPositionRad = pivotMotor.getRotorPosition();

    rollerVoltage = rollerMotor.getMotorVoltage();
    rollerSupplyCurrent = rollerMotor.getSupplyCurrent();
    rollerStatorCurrent = rollerMotor.getStatorCurrent();
    rollerTemperature = rollerMotor.getDeviceTemp();
    rollerVelocityRPS = rollerMotor.getRotorVelocity();

    lvl1blockerVoltage = lvl1blockerMotor.getMotorVoltage();
    lvl1blockerSupplyCurrent = lvl1blockerMotor.getSupplyCurrent();
    lvl1blockerStatorCurrent = lvl1blockerMotor.getStatorCurrent();
    lvl1blockerTemperature = lvl1blockerMotor.getDeviceTemp();
    lvl1blockerVelocityRPS = lvl1blockerMotor.getRotorVelocity();
    lvl1blockerPositionRad = lvl1blockerMotor.getRotorPosition();

    canCoderPositionRad = canCoder.getAbsolutePosition();
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
          pivotPositionRad,
          rollerVoltage,
          rollerSupplyCurrent,
          rollerStatorCurrent,
          rollerTemperature,
          rollerVelocityRPS,
          lvl1blockerVoltage,
          lvl1blockerSupplyCurrent,
          lvl1blockerStatorCurrent,
          lvl1blockerTemperature,
          lvl1blockerVelocityRPS,
          lvl1blockerPositionRad,
          canCoderPositionRad,
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
        pivotPositionRad,
        rollerVoltage,
        rollerSupplyCurrent,
        rollerStatorCurrent,
        rollerTemperature,
        rollerVelocityRPS,
        lvl1blockerVoltage,
        lvl1blockerSupplyCurrent,
        lvl1blockerStatorCurrent,
        lvl1blockerTemperature,
        lvl1blockerVelocityRPS,
        lvl1blockerPositionRad,
        canCoderPositionRad,
        canCoderVelocityRPS,
        canRangeTripped,
        canRangeSignalStrength,
        canRangeDistance);

    pivotMotor.optimizeBusUtilization();
    rollerMotor.optimizeBusUtilization();
    lvl1blockerMotor.optimizeBusUtilization();
    canCoder.optimizeBusUtilization();
    canRange.optimizeBusUtilization();

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        false,
        pivotVoltage,
        pivotSupplyCurrent,
        pivotStatorCurrent,
        pivotTemperature,
        pivotVelocityRPS,
        pivotPositionRad,
        rollerVoltage,
        rollerSupplyCurrent,
        rollerStatorCurrent,
        rollerTemperature,
        rollerVelocityRPS,
        lvl1blockerVoltage,
        lvl1blockerSupplyCurrent,
        lvl1blockerStatorCurrent,
        lvl1blockerTemperature,
        lvl1blockerVelocityRPS,
        lvl1blockerPositionRad,
        canCoderPositionRad,
        canCoderVelocityRPS,
        canRangeTripped,
        canRangeSignalStrength,
        canRangeDistance);
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
                pivotPositionRad),
            pivotVoltage.getValueAsDouble(),
            pivotSupplyCurrent.getValueAsDouble(),
            pivotStatorCurrent.getValueAsDouble(),
            pivotTemperature.getValueAsDouble(),
            pivotVelocityRPS.getValueAsDouble(),
            pivotPositionRad.getValueAsDouble());

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

    inputs.blockerData =
        new Lvl1BlockerData(
            BaseStatusSignal.isAllGood(
                lvl1blockerVoltage,
                lvl1blockerSupplyCurrent,
                lvl1blockerStatorCurrent,
                lvl1blockerTemperature,
                lvl1blockerVelocityRPS,
                lvl1blockerPositionRad),
            lvl1blockerVoltage.getValueAsDouble(),
            lvl1blockerSupplyCurrent.getValueAsDouble(),
            lvl1blockerStatorCurrent.getValueAsDouble(),
            lvl1blockerTemperature.getValueAsDouble(),
            lvl1blockerVelocityRPS.getValueAsDouble(),
            lvl1blockerPositionRad.getValueAsDouble());

    inputs.canCoderData =
        new CanCoderData(
            BaseStatusSignal.isAllGood(canCoderPositionRad, canCoderVelocityRPS),
            canCoderPositionRad.getValueAsDouble(),
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
  public void setLvl1BlockerVoltage(double voltage) {
    lvl1blockerMotor.setControl(lvl1blockerVoltageRequest.withOutput(voltage));
  }

  @Override
  public void setPivotPosition(double positionRad) {
    pivotMotor.setControl(pivotPositionRequest.withPosition(positionRad));
  }

  @Override
  public void setLvl1BlockerPosition(double positionRad) {
    lvl1blockerMotor.setControl(lvl1blockerPositionRequest.withPosition(positionRad));
  }

  @Override
  public boolean checkRollerStalled() {
    return MotorStallDetection.isMotorStalled(
        rollerMotor, IntakeConstants.ROLLER_STALLED_CURRENT, IntakeConstants.ROLLER_STALLED_RPS);
  }
}
