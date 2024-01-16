package frc.robot.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // Measurements in METERS

  // how high your limelight is from the lens
  public static double CAMERA_HEIGHT_METERS = Units.inchesToMeters(36);
  // Camera Angle
  public static double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // FIXME CHANGE WHEN 2024 GAME RELEASES
  public static double APRIL_TAG_HEIGHT_METERS = Units.inchesToMeters(20.5);
  public static double TAPE_HEIGHT_METERS = Units.inchesToMeters(23.5);

  // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  public static Transform3d kRobotToCam =
      new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  // The layout of the AprilTags on the field
  public static AprilTagFieldLayout kTagLayout =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  // FIXME CHANGE WHEN 2024 GAME RELEASES

  public static Pose3d AprilTag1 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(1).get();

  public static Pose3d AprilTag2 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(2).get();

  public static Pose3d AprilTag3 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(3).get();

  public static Pose3d AprilTag4 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(4).get();

  public static Pose3d AprilTag5 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(5).get();

  public static Pose3d AprilTag6 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(6).get();

  public static Pose3d AprilTag7 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(1).get();

  public static Pose3d AprilTag8 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(8).get();

  public static Pose3d AprilTag10 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(10).get();

  public static Pose3d AprilTag11 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(11).get();

  public static Pose3d AprilTag12 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(12).get();

  public static Pose3d AprilTag13 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(13).get();

  public static Pose3d AprilTag14 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(14).get();

  public static Pose3d AprilTag15 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(15).get();

  public static Pose3d AprilTag16 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(16).get();

  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
}
