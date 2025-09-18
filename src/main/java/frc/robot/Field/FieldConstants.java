package frc.robot.Field;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;
import java.util.List;

public class FieldConstants {
  public static record AprilTagStruct(int fiducialId, Pose3d pose) {}

  // Center of AprilTag to the center of pole
  public static final double tagToReef = Units.inchesToMeters(6.469);

  public static final double distanceToTag = Units.inchesToMeters(10);

  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX = Units.inchesToMeters(299.438);

  public static final double halfFieldWidth = fieldWidth / 2;

  public static final AprilTagStruct blueHPSDriverRight =
      new AprilTagStruct(12, VisionConstants.fieldLayout.getTagPose(12).get());
  public static final AprilTagStruct blueHPSDriverLeft =
      new AprilTagStruct(12, VisionConstants.fieldLayout.getTagPose(13).get());

  public static final Integer BLUEPROCESSO_INTEGER = 16;
  public static final AprilTagStruct blueBarge =
      new AprilTagStruct(14, VisionConstants.fieldLayout.getTagPose(14).get());

  // Starting at the side facing driver, clockwise numbering
  public static final ReefFace blueReef1 =
      new ReefFace(new AprilTagStruct(18, VisionConstants.fieldLayout.getTagPose(18).get()));
  public static final ReefFace blueReef2 =
      new ReefFace(new AprilTagStruct(19, VisionConstants.fieldLayout.getTagPose(19).get()));
  public static final ReefFace blueReef3 =
      new ReefFace(new AprilTagStruct(20, VisionConstants.fieldLayout.getTagPose(20).get()));
  public static final ReefFace blueReef4 =
      new ReefFace(new AprilTagStruct(21, VisionConstants.fieldLayout.getTagPose(21).get()));
  public static final ReefFace blueReef5 =
      new ReefFace(new AprilTagStruct(22, VisionConstants.fieldLayout.getTagPose(22).get()));
  public static final ReefFace blueReef6 =
      new ReefFace(new AprilTagStruct(17, VisionConstants.fieldLayout.getTagPose(17).get()));

  public static final AprilTagStruct redHPSDriverRight =
      new AprilTagStruct(12, VisionConstants.fieldLayout.getTagPose(12).get());
  public static final AprilTagStruct redHPSDriverLeft =
      new AprilTagStruct(12, VisionConstants.fieldLayout.getTagPose(13).get());

  public static final Integer REDPROCESSO_INTEGER = 16;
  public static final AprilTagStruct redBarge =
      new AprilTagStruct(14, VisionConstants.fieldLayout.getTagPose(14).get());

  // Starting at the side facing driver, clockwise numbering
  public static final ReefFace redReef1 =
      new ReefFace(new AprilTagStruct(18, VisionConstants.fieldLayout.getTagPose(7).get()));
  public static final ReefFace redReef2 =
      new ReefFace(new AprilTagStruct(19, VisionConstants.fieldLayout.getTagPose(6).get()));
  public static final ReefFace redReef3 =
      new ReefFace(new AprilTagStruct(20, VisionConstants.fieldLayout.getTagPose(11).get()));
  public static final ReefFace redReef4 =
      new ReefFace(new AprilTagStruct(21, VisionConstants.fieldLayout.getTagPose(10).get()));
  public static final ReefFace redReef5 =
      new ReefFace(new AprilTagStruct(22, VisionConstants.fieldLayout.getTagPose(9).get()));
  public static final ReefFace redReef6 =
      new ReefFace(new AprilTagStruct(17, VisionConstants.fieldLayout.getTagPose(8).get()));

  public static final List<ReefFace> blueReefTags =
      List.of(blueReef1, blueReef2, blueReef3, blueReef4, blueReef5, blueReef6);
  public static final List<ReefFace> redReefTags =
      List.of(redReef1, redReef2, redReef3, redReef4, redReef5, redReef6);

  public static final List<AprilTagStruct> blueHPSTags =
      List.of(blueHPSDriverLeft, blueHPSDriverRight);
  public static final List<AprilTagStruct> redHPSTags =
      List.of(redHPSDriverLeft, redHPSDriverRight);
}
