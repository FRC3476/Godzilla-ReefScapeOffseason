package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import frc.robot.Constants;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.LedConstants.LedStrip;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;
import java.util.function.Supplier;

public class LedIOReal implements LedIO {
  private final CANdle candle;
  private LedState currentState = LedState.kCOOrangePure;
  private LedState[] currentPixels =
      new LedState
          [Constants.LedConstants.kCandleLEDCount + Constants.LedConstants.kNonCandleLEDCount];

  public LedIOReal() {
    candle = new CANdle(Constants.LedConstants.ID, Constants.RIO_CANBUS);
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

  private void clearLeds() {
    clearLeft();
    clearRight();
  }

  private void clearLeft() {
    candle.setControl(new EmptyAnimation(0));
    candle.setControl(new EmptyAnimation(2));
    candle.setControl(new EmptyAnimation(4));
    candle.setControl(new EmptyAnimation(6));
  }

  private void clearRight() {
    candle.setControl(new EmptyAnimation(1));
    candle.setControl(new EmptyAnimation(3));
    candle.setControl(new EmptyAnimation(5));
    candle.setControl(new EmptyAnimation(7));
  }

  @Override
  public void writePixels(LedState state, LedStrip strip) {

    switch (strip) {
      case LEFT:
        clearLeft();
        candle.setControl(
            new SolidColor(LedConstants.kLeftLEDStartIdx, LedConstants.kLeftLEDEndIdx)
                .withColor(state.getRGBW()));
        break;
      case RIGHT:
        clearRight();
        candle.setControl(
            new SolidColor(LedConstants.kRightLEDStartIdx, LedConstants.kRightLEDEndIdx)
                .withColor(state.getRGBW()));
        break;
      case BOTH:
        clearLeds();
        candle.setControl(
            new SolidColor(LedConstants.kLeftLEDStartIdx, LedConstants.kLeftLEDEndIdx)
                .withColor(state.getRGBW()));
        candle.setControl(
            new SolidColor(LedConstants.kRightLEDStartIdx, LedConstants.kRightLEDEndIdx)
                .withColor(state.getRGBW()));
        break;
    }
  }

  @Override
  public void writeNumPixels(LedState state, Supplier<Integer> numLeds) {
    clearLeds();
    candle.setControl(new EmptyAnimation(0));
    candle.setControl(new SolidColor(0, numLeds.get()).withColor(state.getRGBW()));
    candle.setControl(new SolidColor(numLeds.get(), 399).withColor(LedState.kOff.getRGBW()));
  }

  private static final LoggedTunableNumber fireSparking =
      new LoggedTunableNumber("LED/Fire Sparking", 0.2);
  private static final LoggedTunableNumber fireCooling =
      new LoggedTunableNumber("LED/Fire Cooling", 0.35);
  private static final LoggedTunableNumber fireFrameRate =
      new LoggedTunableNumber("LED/Fire Frame Rate", 60);

  @Override
  public void fire() {
    clearLeds();
    candle.setControl(
        new FireAnimation(LedConstants.kLeftLEDStartIdx, LedConstants.kLeftLEDEndIdx)
            .withSparking(fireSparking.get())
            .withCooling(fireCooling.get())
            .withFrameRate(fireFrameRate.get())
            .withSlot(0));
    candle.setControl(
        new FireAnimation(LedConstants.kRightLEDStartIdx, LedConstants.kRightLEDEndIdx)
            .withSparking(fireSparking.get())
            .withCooling(fireCooling.get())
            .withFrameRate(fireFrameRate.get())
            .withDirection(AnimationDirectionValue.Backward)
            .withSlot(1));
  }

  private static final LoggedTunableNumber larsonBounceMode =
      new LoggedTunableNumber("LED/Larson Mode", 2);
  private static final LoggedTunableNumber larsonSize =
      new LoggedTunableNumber("LED/Larson Size", 5);
  private static final LoggedTunableNumber larsonFrameRate =
      new LoggedTunableNumber("LED/Larson Frame Rate", 30);

  @Override
  public void larson(LedState state) {
    clearLeds();
    candle.setControl(
        new LarsonAnimation(LedConstants.kLeftLEDStartIdx, LedConstants.kLeftLEDEndIdx)
            .withBounceMode(
                (int) larsonBounceMode.get() == 0
                    ? LarsonBounceValue.Front
                    : ((int) larsonBounceMode.get() == 1
                        ? LarsonBounceValue.Center
                        : LarsonBounceValue.Back))
            .withColor(state.getRGBW())
            .withFrameRate(larsonFrameRate.get())
            .withSize((int) larsonSize.get())
            .withSlot(0));
    candle.setControl(
        new LarsonAnimation(LedConstants.kRightLEDEndIdx, LedConstants.kRightLEDStartIdx)
            .withBounceMode(
                (int) larsonBounceMode.get() == 0
                    ? LarsonBounceValue.Front
                    : ((int) larsonBounceMode.get() == 1
                        ? LarsonBounceValue.Center
                        : LarsonBounceValue.Back))
            .withColor(state.getRGBW())
            .withFrameRate(larsonFrameRate.get())
            .withSize((int) larsonSize.get())
            .withSlot(1));
  }

  private static final LoggedTunableNumber colorflowFrameRate =
      new LoggedTunableNumber("LED/Colorflow Frame Rate", 30);

  @Override
  public void colorflowCO() {
    clearLeds();
    candle.setControl(
        new ColorFlowAnimation(LedConstants.kLeftLEDStartIdx, 15)
            .withColor(LedState.kCOTealLed.getRGBW())
            .withFrameRate(colorflowFrameRate.get())
            .withDirection(AnimationDirectionValue.Forward)
            .withSlot(0));
    candle.setControl(
        new ColorFlowAnimation(LedConstants.kRightLEDStartIdx, 31)
            .withColor(LedState.kCOOrangeLed.getRGBW())
            .withFrameRate(colorflowFrameRate.get())
            .withDirection(AnimationDirectionValue.Forward)
            .withSlot(1));
    candle.setControl(
        new ColorFlowAnimation(16, LedConstants.kLeftLEDEndIdx)
            .withColor(LedState.kCOOrangeLed.getRGBW())
            .withFrameRate(colorflowFrameRate.get())
            .withDirection(AnimationDirectionValue.Backward)
            .withSlot(2));
    candle.setControl(
        new ColorFlowAnimation(32, LedConstants.kRightLEDEndIdx)
            .withColor(LedState.kCOTealLed.getRGBW())
            .withFrameRate(colorflowFrameRate.get())
            .withDirection(AnimationDirectionValue.Backward)
            .withSlot(3));
  }

  private static final LoggedTunableNumber twinkleMax =
      new LoggedTunableNumber("LED/Twinkle Max", 1);
  private static final LoggedTunableNumber twinkleFrameRate =
      new LoggedTunableNumber("LED/Twinkle Frame Rate", 30);

  @Override
  public void twinkle(LedState state, boolean off) {
    clearLeds();
    candle.setControl(
        off
            ? new TwinkleAnimation(LedConstants.kLeftLEDStartIdx, LedConstants.kLeftLEDEndIdx)
                .withColor(state.getRGBW())
                .withFrameRate(twinkleFrameRate.get())
                .withMaxLEDsOnProportion(twinkleMax.get())
                .withSlot(0)
            : new TwinkleOffAnimation(LedConstants.kLeftLEDStartIdx, LedConstants.kLeftLEDEndIdx)
                .withColor(state.getRGBW())
                .withFrameRate(twinkleFrameRate.get())
                .withMaxLEDsOnProportion(twinkleMax.get())
                .withSlot(0));
    candle.setControl(
        off
            ? new TwinkleAnimation(LedConstants.kRightLEDStartIdx, LedConstants.kRightLEDEndIdx)
                .withColor(state.getRGBW())
                .withFrameRate(twinkleFrameRate.get())
                .withMaxLEDsOnProportion(twinkleMax.get())
                .withSlot(1)
            : new TwinkleOffAnimation(LedConstants.kRightLEDStartIdx, LedConstants.kRightLEDEndIdx)
                .withColor(state.getRGBW())
                .withFrameRate(twinkleFrameRate.get())
                .withMaxLEDsOnProportion(twinkleMax.get())
                .withSlot(1));
  }

  @Override
  public void rainbow() {
    clearLeds();
    candle.setControl(
        new RainbowAnimation(LedConstants.kLeftLEDStartIdx, LedConstants.kLeftLEDEndIdx)
            .withSlot(0));
    candle.setControl(
        new RainbowAnimation(LedConstants.kRightLEDStartIdx, LedConstants.kRightLEDEndIdx)
            .withDirection(AnimationDirectionValue.Backward)
            .withSlot(1));
  }

  @Override
  public void blink(LedState state, double duration, LedStrip strip) {

    switch (strip) {
      case LEFT:
        clearLeft();
        candle.setControl(
            new StrobeAnimation(LedConstants.kLeftLEDStartIdx, LedConstants.kLeftLEDEndIdx)
                .withColor(state.getRGBW())
                .withUpdateFreqHz(1 / duration)
                .withSlot(0));
        break;
      case RIGHT:
        clearRight();
        candle.setControl(
            new StrobeAnimation(LedConstants.kRightLEDStartIdx, LedConstants.kRightLEDEndIdx)
                .withColor(state.getRGBW())
                .withUpdateFreqHz(1 / duration)
                .withSlot(1));
        break;
      case BOTH:
        clearLeds();
        candle.setControl(
            new StrobeAnimation(LedConstants.kLeftLEDStartIdx, LedConstants.kLeftLEDEndIdx)
                .withColor(state.getRGBW())
                .withUpdateFreqHz(1 / duration)
                .withSlot(0));
        candle.setControl(
            new StrobeAnimation(LedConstants.kRightLEDStartIdx, LedConstants.kRightLEDEndIdx)
                .withColor(state.getRGBW())
                .withUpdateFreqHz(1 / duration)
                .withSlot(1));
        break;
    }
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
