package frc.robot.subsystems.led;

import com.ctre.phoenix6.signals.RGBWColor;

public class LedState {
  public static final LedState kRed = new LedState(255, 0, 0);
  public static final LedState kBlue = new LedState(0, 0, 255);
  public static final LedState kCyan = new LedState(0, 255, 255);
  public static final LedState kGreen = new LedState(0, 255, 0);
  public static final LedState kYellow = new LedState(255, 215, 0);
  public static final LedState kOrange = new LedState(255, 80, 0);
  public static final LedState kPurple = new LedState(255, 0, 255);
  public static final LedState kWhite = new LedState(255, 255, 255);
  public static final LedState kGray = new LedState(128, 128, 128);
  public static final LedState kPink = new LedState(255, 0, 100);
  public static final LedState kCOTealPure = new LedState(32, 146, 153);
  public static final LedState kCOOrangePure = new LedState(255, 122, 28);
  public static final LedState kCOTealLed = new LedState(0, 100, 35);
  public static final LedState kCOOrangeLed = new LedState(255, 30, 0);

  public static final LedState kOff = new LedState(0, 0, 0); // No Color
  public static final LedState kLowBattery = kYellow;
  public static final LedState kGoodBattery = kGreen;
  public static final LedState[] kRainbow = {
    kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kOff, kRed, kOrange, kYellow,
    kGreen, kCyan, kBlue, kPurple, kOff, kOff
  };

  public static final LedState kCoralMode = kCOOrangeLed;
  public static final LedState kAlgaeMode = kCOTealLed;
  public static final LedState kCoralManual = kWhite;

  // 3 leds for l2, 7 leds for l3, full for l4
  public static final LedState[] kL2StagingLeds = {
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kOff,
    kOff,
    kOff,
    kOff,
    kOff,
    kOff
  };
  public static final LedState[] kL3StagingLeds = {
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kCoralMode,
    kOff,
    kOff,
    kOff
  };

  public static final LedState[] kL2StagingManualLeds = {
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kOff,
    kOff,
    kOff,
    kOff,
    kOff,
    kOff
  };
  public static final LedState[] kL3StagingManualLeds = {
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kCoralManual,
    kOff,
    kOff,
    kOff
  };

  public static final LedState[] kAlgaeL2Leds = {
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kOff,
    kOff,
    kOff,
    kOff,
    kOff,
    kOff
  };
  public static final LedState[] kAlgaeL3Leds = {
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kAlgaeMode,
    kOff,
    kOff,
    kOff
  };

  public int blue;
  public int green;
  public int red;

  public LedState() {
    blue = 0;
    green = 0;
    red = 0;
  }

  public LedState(int r, int g, int b) {
    blue = b;
    green = g;
    red = r;
  }

  public RGBWColor getRGBW() {
    return new RGBWColor(this.red, this.green, this.blue);
  }

  public void copyFrom(LedState other) {
    this.blue = other.blue;
    this.green = other.green;
    this.red = other.red;
  }

  @Override
  public boolean equals(Object other) {
    if (other == null) {
      return false;
    }

    if (other.getClass() != this.getClass()) {
      return false;
    }

    LedState s = (LedState) other;
    return this.blue == s.blue && this.red == s.red && this.green == s.green;
  }

  @Override
  public String toString() {
    String redString = Integer.toHexString(red);
    if (redString.length() == 1) {
      redString = "0" + redString;
    }
    String greenString = Integer.toHexString(green);
    if (greenString.length() == 1) {
      greenString = "0" + greenString;
    }
    String blueString = Integer.toHexString(blue);
    if (blueString.length() == 1) {
      blueString = "0" + blueString;
    }
    return "#" + redString + greenString + blueString;
  }
}
