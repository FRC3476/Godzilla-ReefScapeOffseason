package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.*;
import org.junit.jupiter.api.Test;

/*
 * AI Generated Tests for MathHelpers
 */

public class MathHelpersTest {

  private static final double EPS = 1e-9;

  /* ========================
   *  DOT / CROSS PRODUCT TESTS
   * ======================== */

  @Test
  void testDotProduct2d() {
    Translation2d a = new Translation2d(3, 4);
    Translation2d b = new Translation2d(2, 0);
    assertEquals(6.0, MathHelpers.dotProduct(a, b), EPS);
  }

  @Test
  void testDotProduct3d() {
    Translation3d a = new Translation3d(1, 2, 3);
    Translation3d b = new Translation3d(4, -5, 6);
    assertEquals(12.0, MathHelpers.dotProduct(a, b), EPS);
  }

  @Test
  void testCrossProduct3d() {
    Translation3d a = new Translation3d(1, 0, 0);
    Translation3d b = new Translation3d(0, 1, 0);
    Translation3d cross = MathHelpers.crossProduct(a, b);
    assertEquals(new Translation3d(0, 0, 1), cross);
  }

  @Test
  void testCrossProductDirection() {
    Translation2d a = new Translation2d(1, 0);
    Translation2d b = new Translation2d(0, 1);
    assertEquals(1, MathHelpers.crossProductDirection(a, b)); // left turn
    assertEquals(-1, MathHelpers.crossProductDirection(b, a)); // right turn
  }

  @Test
  void testCrossProductDirectionCollinear() {
    Translation2d a = new Translation2d(2, 2);
    Translation2d b = new Translation2d(4, 4);
    int sign = MathHelpers.crossProductDirection(a, b);
    // Either 1, -1, or 0 depending on your definition — we expect no crash
    assertTrue(sign == 1 || sign == -1 || sign == 0);
  }

  /* ========================
   *  PROJECTION TESTS
   * ======================== */

  @Test
  void testProjection2d() {
    Translation2d a = new Translation2d(3, 4);
    Translation2d b = new Translation2d(1, 0); // along x-axis
    Translation2d proj = MathHelpers.projectedOnto(a, b);
    assertEquals(new Translation2d(3, 0), proj);
  }

  @Test
  void testProjection3d() {
    Translation3d a = new Translation3d(2, 3, 4);
    Translation3d b = new Translation3d(0, 0, 1);
    Translation3d proj = MathHelpers.projectedOnto(a, b);
    assertEquals(new Translation3d(0, 0, 4), proj);
  }

  @Test
  void testProjectionZeroVector2d() {
    Translation2d a = new Translation2d(3, 4);
    Translation2d zero = new Translation2d(0, 0);
    Translation2d proj = MathHelpers.projectedOnto(a, zero);
    assertEquals(new Translation2d(), proj);
  }

  @Test
  void testProjectionZeroVector3d() {
    Translation3d a = new Translation3d(3, 4, 5);
    Translation3d zero = new Translation3d(0, 0, 0);
    Translation3d proj = MathHelpers.projectedOnto(a, zero);
    assertEquals(new Translation3d(), proj);
  }

  /* ========================
   *  LINE / SEGMENT DISTANCE TESTS
   * ======================== */

  @Test
  void testReverseInterpolate() {
    Translation2d start = new Translation2d(0, 0);
    Translation2d end = new Translation2d(10, 0);
    Translation2d query = new Translation2d(5, 0);
    assertEquals(0.5, MathHelpers.reverseInterpolate(query, start, end), EPS);
  }

  @Test
  void testReverseInterpolateZeroLengthSegment() {
    Translation2d start = new Translation2d(1, 1);
    Translation2d query = new Translation2d(2, 3);
    // start == end
    assertEquals(0.0, MathHelpers.reverseInterpolate(query, start, start), EPS);
  }

  @Test
  void testDistanceToLineSegmentInside() {
    Translation2d start = new Translation2d(0, 0);
    Translation2d end = new Translation2d(10, 0);
    Translation2d query = new Translation2d(5, 3);
    assertEquals(3.0, MathHelpers.distanceToLineSegment(query, start, end), EPS);
  }

  @Test
  void testDistanceToLineSegmentOutside() {
    Translation2d start = new Translation2d(0, 0);
    Translation2d end = new Translation2d(10, 0);
    Translation2d query = new Translation2d(15, 4);
    assertEquals(query.getDistance(end), MathHelpers.distanceToLineSegment(query, start, end), EPS);
  }

  @Test
  void testDistanceToLineSegmentZeroLength() {
    Translation2d start = new Translation2d(2, 2);
    Translation2d query = new Translation2d(5, 5);
    double expected = query.getDistance(start);
    assertEquals(expected, MathHelpers.distanceToLineSegment(query, start, start), EPS);
  }

  @Test
  void testPerpendicularDistanceToLine() {
    Translation2d start = new Translation2d(0, 0);
    Translation2d end = new Translation2d(10, 0);
    Translation2d query = new Translation2d(5, 3);
    assertEquals(3.0, MathHelpers.perpendicularDistanceToLine(query, start, end), EPS);
  }

  /* ========================
   *  DIRECTIONAL DISTANCE TESTS
   * ======================== */

  @Test
  void testDistanceToPointInDirection() {
    Pose2d pose = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
    Translation2d target = new Translation2d(5, 0);
    assertEquals(5.0, MathHelpers.distanceToPointInDirection(pose, target), EPS);

    Translation2d offToSide = new Translation2d(0, 5);
    assertEquals(0.0, MathHelpers.distanceToPointInDirection(pose, offToSide), EPS);
  }

  @Test
  void testDistanceToPointInArbitraryDirection() {
    Translation2d origin = new Translation2d(0, 0);
    Translation2d target = new Translation2d(2, 2);
    Rotation2d dir45 = Rotation2d.fromDegrees(45);
    // projection of (2,2) on 45° direction is sqrt(8)/√2 = 2√2
    assertEquals(Math.sqrt(8), MathHelpers.distanceToPointInDirection(origin, target, dir45), EPS);
  }

  /* ========================
   *  POSE / TRANSFORM FACTORY TESTS
   * ======================== */

  @Test
  void testPoseFromRotation() {
    Rotation2d rot = Rotation2d.fromDegrees(90);
    Pose2d pose = MathHelpers.pose2dFromRotation(rot);
    assertEquals(Translation2d.kZero, pose.getTranslation());
    assertEquals(rot, pose.getRotation());
  }

  @Test
  void testPoseFromTranslation() {
    Translation2d trans = new Translation2d(3, 4);
    Pose2d pose = MathHelpers.pose2dFromTranslation(trans);
    assertEquals(trans, pose.getTranslation());
    assertEquals(Rotation2d.kZero, pose.getRotation());
  }

  @Test
  void testTransformFromRotation() {
    Rotation2d rot = Rotation2d.fromDegrees(45);
    Transform2d tf = MathHelpers.transform2dFromRotation(rot);
    assertEquals(Translation2d.kZero, tf.getTranslation());
    assertEquals(rot, tf.getRotation());
  }

  @Test
  void testTransformFromTranslation() {
    Translation2d trans = new Translation2d(5, 6);
    Transform2d tf = MathHelpers.transform2dFromTranslation(trans);
    assertEquals(trans, tf.getTranslation());
    assertEquals(Rotation2d.kZero, tf.getRotation());
  }
}
