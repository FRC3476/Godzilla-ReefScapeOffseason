package frc.lib;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.util.Arrays;

public class AprilTagLayout {
  // April Tag Layout
  public static final AprilTagFieldLayout kAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  public static final int[] kAllowedTagIDs = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};
  public static final AprilTagFieldLayout kAprilTagLayoutReefsOnly =
      new AprilTagFieldLayout(
          kAprilTagLayout.getTags().stream()
              .filter(tag -> Arrays.stream(kAllowedTagIDs).anyMatch(element -> element == tag.ID))
              .toList(),
          kAprilTagLayout.getFieldLength(),
          kAprilTagLayout.getFieldWidth());

  public static final double kFieldWidthMeters = kAprilTagLayout.getFieldWidth();
  public static final double kFieldLengthMeters = kAprilTagLayout.getFieldLength();
}
