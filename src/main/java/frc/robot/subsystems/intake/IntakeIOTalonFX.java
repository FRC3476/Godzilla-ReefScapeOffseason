package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX pivotMotor;
  private final TalonFX rollerMotor;
  private final CANcoder canCoder;
  private final CANrange canRange;

  private final VoltageOut pivotVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut rollerVoltageRequest = new VoltageOut(0.0);
  private final PositionVoltage pivotPositionRequest = new PositionVoltage(0.0);

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

  // CANCoder status signals
  private final StatusSignal<Angle> canCoderPositionRad;
  private final StatusSignal<AngularVelocity> canCoderVelocityRPS;

  // CANRange status signals
  private final StatusSignal<Boolean> canRangeTripped;
  private final StatusSignal<Double> canRangeSignalStrength;
  private final StatusSignal<Distance> canRangeDistance;

  public IntakeIOTalonFX() {
    // Initialize hardware
    pivotMotor = new TalonFX(IntakeConstants.intakePivotID);
    rollerMotor = new TalonFX(IntakeConstants.intakeRollerID);
    canCoder = new CANcoder(IntakeConstants.CANCODER_ID);
    canRange = new CANrange(IntakeConstants.CANRANGE_ID);

    // Configure pivot motor
    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.MAX_CURRENT_LIMIT;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.MAX_CURRENT_LIMIT;
    pivotConfig.Slot0.kP = IntakeConstants.KP;
    pivotConfig.Slot0.kI = IntakeConstants.KI;
    pivotConfig.Slot0.kD = IntakeConstants.KD;
    pivotConfig.Slot0.kG = IntakeConstants.KG;
    pivotConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.MAX_ACCEL;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.MAX_VELOCITY;
    pivotMotor.getConfigurator().apply(pivotConfig);

    // Configure roller motor
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.MAX_CURRENT_LIMIT;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.MAX_CURRENT_LIMIT;
    rollerMotor.getConfigurator().apply(rollerConfig);

    // Configure CANCoder
    var canCoderConfig = new CANcoderConfiguration();
    canCoder.getConfigurator().apply(canCoderConfig);

    // Configure CANRange
    var canRangeConfig = new CANrangeConfiguration();
    canRangeConfig.ProximityParams.ProximityThreshold = 0.05; // 5cm detection threshold
    canRangeConfig.ProximityParams.ProximityHysteresis = 0.01; // 1cm hysteresis
    canRange.getConfigurator().apply(canRangeConfig);

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

    canCoderPositionRad = canCoder.getAbsolutePosition();
    canCoderVelocityRPS = canCoder.getVelocity();

    canRangeTripped = canRange.getIsDetected();
    canRangeSignalStrength = canRange.getSignalStrength();
    canRangeDistance = canRange.getDistance();
  }

  @Override
  public void runPivotVolts(double volts) {
    pivotMotor.setControl(pivotVoltageRequest.withOutput(volts));
  }

}
