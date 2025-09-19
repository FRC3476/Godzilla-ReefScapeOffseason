package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.PhoenixUtil;

public class LedIOHardware implements LedIO {
  private final CANdle candle;
  private LedState currentState = LedState.kBlue;
  private LedState[] currentPixels =
      new LedState
          [Constants.LEDConstants.kCandleLEDCount + Constants.LEDConstants.kNonCandleLEDCount];

  public LedIOHardware() {
    if (Robot.isReal()) {
      candle = new CANdle(Constants.LEDConstants.ID);
      LEDConfigs ledConfigs =
          new LEDConfigs().withBrightnessScalar(1.0).withStripType(StripTypeValue.RGB);
      CANdleConfiguration candleConfiguration = new CANdleConfiguration().withLED(ledConfigs);
      PhoenixUtil.tryUntilOk(5, () -> candle.getConfigurator().apply(candleConfiguration));
    } else {
      candle = null;
    }
  }

  public LedState getCurrentState() {
    return currentState;
  }

  public LedState[] getCurrentPixels() {
    return currentPixels;
  }

  @Override
  public void writePixels(LedState state) {
    if (state == null) state = LedState.kOff;
    currentState = state;
    if (candle != null) candle.setControl(new SolidColor(0, 399).withColor(state.getRGBW()));
  }

  @Override
  public void writePixels(LedState[] pixels) {
    // do not write empty data
    if (pixels == null || pixels.length == 0) {
      return;
    }

    LedState run = pixels[0];
    int runStart = 0;
    for (int i = 0; i < pixels.length; i++) {
      if (pixels[i] == null) pixels[i] = LedState.kOff;
      if (!run.equals(pixels[i])) {
        if (candle != null)
          candle.setControl(new SolidColor(runStart, i - runStart).withColor(run.getRGBW()));
        runStart = i;
        run = pixels[i];
        currentPixels[i] = run;
      }
    }

    if (candle != null)
      candle.setControl(
          new SolidColor(runStart, pixels.length - runStart).withColor(run.getRGBW()));
  }
}
