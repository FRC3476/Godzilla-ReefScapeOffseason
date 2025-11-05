package frc.robot.util;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Color {
  public static final Color kRed = new Color(255, 0, 0);
  public static final Color kBlue = new Color(0, 0, 255);
  public static final Color kCyan = new Color(0, 255, 255);
  public static final Color kGreen = new Color(0, 255, 0);
  public static final Color kYellow = new Color(255, 215, 0);
  public static final Color kOrange = new Color(255, 80, 0);
  public static final Color kPurple = new Color(255, 0, 255);
  public static final Color kWhite = new Color(255, 255, 255);
  public static final Color kGray = new Color(128, 128, 128);
  public static final Color kPink = new Color(255, 0, 100);
  public static final Color kCOTealPure = new Color(32, 146, 153);
  public static final Color kCOOrangePure = new Color(255, 122, 28);
  public static final Color kCOTealLed = new Color(0, 100, 35);
  public static final Color kCOOrangeLed = new Color(255, 30, 0);

  public static final Color kOff = new Color(0, 0, 0); // No Color
  public static final Color kLowBattery = kYellow;
  public static final Color kGoodBattery = kGreen;
  public static final Color[] kRainbow = {
    kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kOff, kRed, kOrange, kYellow,
    kGreen, kCyan, kBlue, kPurple, kOff, kOff
  };

  public static final Color kCoralMode = kCOOrangeLed;
  public static final Color kAlgaeMode = kCOTealLed;
  public static final Color kCoralManual = kWhite;

  public int blue;
  public int green;
  public int red;

  public Color() {
    blue = 0;
    green = 0;
    red = 0;
  }

  public Color(int r, int g, int b) {
    blue = b;
    green = g;
    red = r;
  }

  public RGBWColor getRGBW() {
    return new RGBWColor(this.red, this.green, this.blue);
  }

  public Color8Bit getColor8Bit() {
    return new Color8Bit(this.red, this.green, this.blue);
  }

  public void copyFrom(Color other) {
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

    Color s = (Color) other;
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
