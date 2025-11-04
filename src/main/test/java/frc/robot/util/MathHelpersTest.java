package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.*;
import org.junit.jupiter.api.Test;

/*
 * AI Generated Tests for MathHelpers
 */
 
 /**
  * Unit tests for MathHelpers.
  * 
  * Run with:
  *    ./gradlew test --tests "frc.robot.util.MathHelpersTest"
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
     assertEquals(1, MathHelpers.crossProductDirection(a, b));
     assertEquals(-1, MathHelpers.crossProductDirection(b, a));
     assertEquals(0, MathHelpers.crossProductDirection(a, new Translation2d(2, 0))); // collinear
   }
 
   /* ========================
    *  PROJECTION TESTS
    * ======================== */
 
   @Test
   void testProjection2d() {
     Translation2d a = new Translation2d(3, 4);
     Translation2d b = new Translation2d(1, 0);
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
     Translation2d zero = Translation2d.kZero;
     Translation2d proj = MathHelpers.projectedOnto(a, zero);
     assertEquals(Translation2d.kZero, proj);
   }
 
   /* ========================
    *  LINE SEGMENT / DISTANCE TESTS
    * ======================== */
 
   @Test
   void testReverseInterpolate() {
     Translation2d start = new Translation2d(0, 0);
     Translation2d end = new Translation2d(10, 0);
     Translation2d query = new Translation2d(5, 0);
     assertEquals(0.5, MathHelpers.reverseInterpolate(query, start, end), EPS);
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
     assertEquals(Math.sqrt(8), MathHelpers.distanceToPointInDirection(origin, target, dir45), EPS);
   }
 
   /* ========================
    *  OFFSET POSE TESTS
    * ======================== */
 
   @Test
   void testParallelOffsetPose() {
     Pose2d base = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
     Pose2d offset = MathHelpers.getParallelOffsetPose(base, 2.0);
     assertEquals(new Translation2d(0, 2), offset.getTranslation());
     assertEquals(base.getRotation(), offset.getRotation());
   }
 
   @Test
   void testPerpendicularOffsetPose() {
     Pose2d base = new Pose2d(new Translation2d(1, 1), Rotation2d.fromDegrees(90));
     Pose2d offset = MathHelpers.getPerpendicularOffsetPose(base, 2.0);
     // 90° rotation → offset moves upward along y-axis
     assertEquals(new Translation2d(1, 3), offset.getTranslation());
     assertEquals(base.getRotation(), offset.getRotation());
   }
 
   @Test
   void testCombinedOffsetPose() {
     Pose2d base = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
     Pose2d offset = MathHelpers.getOffsetPose(base, 2.0, 3.0);
     assertEquals(new Translation2d(3, 2), offset.getTranslation());
     assertEquals(base.getRotation(), offset.getRotation());
   }
 
   @Test
   void testOffsetPoseWithDirection() {
     Pose2d base = new Pose2d(new Translation2d(1, 1), Rotation2d.fromDegrees(0));
     Pose2d offset = MathHelpers.getOffsetPose(base, 5.0, Rotation2d.fromDegrees(90));
     // Should move upward 5 meters
     assertEquals(new Translation2d(1, 6), offset.getTranslation());
   }
 
   /* ========================
    *  PARALLEL / PERP ERROR TESTS
    * ======================== */
 
   @Test
   void testParallelErrorZero() {
     Pose2d origin = new Pose2d(new Translation2d(0, 0), Rotation2d.kZero);
     Pose2d target = new Pose2d(new Translation2d(5, 0), Rotation2d.kZero);
     assertEquals(0.0, MathHelpers.getParallelError(origin, target), EPS);
   }
 
   @Test
   void testPerpendicularErrorMagnitude() {
     Pose2d origin = new Pose2d(new Translation2d(0, 0), Rotation2d.kZero);
     Pose2d target = new Pose2d(new Translation2d(0, 5), Rotation2d.kZero);
     // Offset is purely perpendicular (upward)
     double perpendicularError = MathHelpers.getPerpendicularError(origin, target);
     assertTrue(Math.abs(perpendicularError - 5.0) < EPS);
   }
 
   /* ========================
    *  FACTORY METHOD TESTS
    * ======================== */
 
   @Test
   void testPoseFromRotation() {
     Rotation2d rot = Rotation2d.fromDegrees(45);
     Pose2d pose = MathHelpers.pose2dFromRotation(rot);
     assertEquals(Translation2d.kZero, pose.getTranslation());
     assertEquals(rot, pose.getRotation());
   }
 
   @Test
   void testPoseFromTranslation() {
     Translation2d trans = new Translation2d(2, 3);
     Pose2d pose = MathHelpers.pose2dFromTranslation(trans);
     assertEquals(trans, pose.getTranslation());
     assertEquals(Rotation2d.kZero, pose.getRotation());
   }
 
   @Test
   void testTransformFromRotation() {
     Rotation2d rot = Rotation2d.fromDegrees(30);
     Transform2d tf = MathHelpers.transform2dFromRotation(rot);
     assertEquals(Translation2d.kZero, tf.getTranslation());
     assertEquals(rot, tf.getRotation());
   }
 
   @Test
   void testTransformFromTranslation() {
     Translation2d trans = new Translation2d(1, 2);
     Transform2d tf = MathHelpers.transform2dFromTranslation(trans);
     assertEquals(trans, tf.getTranslation());
     assertEquals(Rotation2d.kZero, tf.getRotation());
   }
 }
 