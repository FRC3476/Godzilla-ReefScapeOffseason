package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class LedIOReal implements LedIO {
  private final CANdle candle;
  private LedState currentState = LedState.kCOOrangePure;
  private LedState[] currentPixels =
      new LedState
          [Constants.LedConstants.kCandleLEDCount + Constants.LedConstants.kNonCandleLEDCount];

  public LedIOReal() {
    candle = new CANdle(Constants.LedConstants.ID, Constants.DRIVE_CANIVORE);
    LEDConfigs ledConfigs =
        new LEDConfigs().withBrightnessScalar(.2).withStripType(StripTypeValue.RGB);
    CANdleConfiguration candleConfiguration = new CANdleConfiguration().withLED(ledConfigs);
    PhoenixUtil.tryUntilOk(5, () -> candle.getConfigurator().apply(candleConfiguration));
  }

  public LedState getCurrentState() {
    return currentState;
  }

  public LedState[] getCurrentPixels() {
    return currentPixels;
  }

  // @Override
  // public void writePixels(LedState state) {
  //   if (state == null) state = LedState.kOff;
  //   currentState = state;
  //   if (candle != null) candle.setControl(new SolidColor(0, 399).withColor(state.getRGBW()));
  // }

  @Override
  public void writePixels(LedState state) {
    candle.setControl(new EmptyAnimation(0));
    candle.setControl(new SolidColor(0, 399).withColor(state.getRGBW()));
  }

  @Override
  public void fire() {
    if (candle != null)
      candle.setControl(new FireAnimation(0, 399).withSparking(0.1).withCooling(0.7));
  }

  @Override
  public void rainbow() {
    if (candle != null) candle.setControl(new RainbowAnimation(0, 399));
  }

  @Override
  public void blink(LedState state, double duration) {
    candle.setControl(
        new StrobeAnimation(0, 399).withColor(state.getRGBW()).withUpdateFreqHz(1 / duration));
  }

  @Override
  public void percentageFull(double percent, LedState state) {
    candle.setControl(new EmptyAnimation(0));
    // candle.setControl(
    //     new SolidColor(0, (int) Math.round(7 + Constants.LedConstants.kNonCandleLEDCount *
    // percent))
    //         .withColor(state.getRGBW()));
    // candle.setControl(
    //     new SolidColor(
    //             (int) Math.round(7 + Constants.LedConstants.kNonCandleLEDCount * percent),
    //             Constants.LedConstants.kNonCandleLEDCount)
    //         .withColor(LedState.kOff.getRGBW()));
    candle.setControl(new SolidColor(0, 20).withColor(state.getRGBW()));
    candle.setControl(new SolidColor(20, 399).withColor(LedState.kOff.getRGBW()));
  }

  // @Override
  // public void writePixels(LedState[] pixels) {
  //   // do not write empty data
  //   if (pixels == null || pixels.length == 0) {
  //     return;
  //   }

  //   LedState run = pixels[0];
  //   int runStart = 0;
  //   for (int i = 0; i < pixels.length; i++) {
  //     if (pixels[i] == null) pixels[i] = LedState.kOff;
  //     if (!run.equals(pixels[i])) {
  //       if (candle != null)
  //         candle.setControl(new SolidColor(runStart, i - runStart).withColor(run.getRGBW()));
  //       runStart = i;
  //       run = pixels[i];
  //       currentPixels[i] = run;
  //     }
  //   }

  //   if (candle != null)
  //     candle.setControl(
  //         new SolidColor(runStart, pixels.length - runStart).withColor(run.getRGBW()));
  // }
}
