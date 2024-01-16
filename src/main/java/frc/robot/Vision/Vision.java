package frc.robot.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ModeSet.Mode;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private double lastEstTimestamp = 0;

  // Simulation
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;

  public Vision() {
    camera = new PhotonCamera("limelight");

    photonEstimator =
        new PhotonPoseEstimator(
            VisionConstants.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            VisionConstants.kRobotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // ----- Simulation
    if (Mode.SIM != null) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.

      visionSim.addAprilTags(VisionConstants.kTagLayout);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      cameraSim = new PhotonCameraSim(camera, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(cameraSim, VisionConstants.kRobotToCam);

      cameraSim.enableDrawWireframe(true);
    }
  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (Mode.SIM != null) {
      visionEst.ifPresentOrElse(
          est ->
              getSimDebugField()
                  .getObject("VisionEstimation")
                  .setPose(est.estimatedPose.toPose2d()),
          () -> {
            if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
          });
    }
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = VisionConstants.kSingleTagStdDevs;
    var targets = getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public PhotonPipelineResult getLimelightResult() {
    return this.camera.getLatestResult();
  }

  /**
   * @return if the limelight detects a target or not
   */
  public boolean limelightTargets() {
    return getLimelightResult().hasTargets();
  }

  /**
   * @return The best target that Limelight sees.
   */
  public PhotonTrackedTarget getLimelightTarget() {
    return getLimelightResult().getBestTarget();
  }

  /**
   * @param tape true for retroreflective tape false for april tag
   * @return the distance between limelight and target(tape or tag)
   */
  public double calculateLimelightRange(boolean tape) {
    return PhotonUtils.calculateDistanceToTargetMeters(
        VisionConstants.CAMERA_HEIGHT_METERS,
        tape
            ? VisionConstants.TAPE_HEIGHT_METERS
            : VisionConstants.APRIL_TAG_HEIGHT_METERS, // Ternary operator
        VisionConstants.CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(getLimelightResult().getBestTarget().getPitch()));
  }

  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  public Pose2d getPose(Pose2d robotSimPose) {
    return robotSimPose;
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Mode.SIM != null) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!(Mode.SIM != null)) return null;
    return visionSim.getDebugField();
  }

  // used for logging april tag poses
  @Override
  public void periodic() {
    Logger.recordOutput("Field/April Tags/April Tag 1", VisionConstants.AprilTag1);
    Logger.recordOutput("Field/April Tags/April Tag 2", VisionConstants.AprilTag2);
    Logger.recordOutput("Field/April Tags/April Tag 3", VisionConstants.AprilTag3);
    Logger.recordOutput("Field/April Tags/April Tag 4", VisionConstants.AprilTag4);
    Logger.recordOutput("Field/April Tags/April Tag 5", VisionConstants.AprilTag5);
    Logger.recordOutput("Field/April Tags/April Tag 6", VisionConstants.AprilTag6);
    Logger.recordOutput("Field/April Tags/April Tag 7", VisionConstants.AprilTag7);
    Logger.recordOutput("Field/April Tags/April Tag 8", VisionConstants.AprilTag8);
    Logger.recordOutput("Field/April Tags/April Tag 10", VisionConstants.AprilTag1);
    Logger.recordOutput("Field/April Tags/April Tag 11", VisionConstants.AprilTag1);
    Logger.recordOutput("Field/April Tags/April Tag 12", VisionConstants.AprilTag2);
    Logger.recordOutput("Field/April Tags/April Tag 13", VisionConstants.AprilTag3);
    Logger.recordOutput("Field/April Tags/April Tag 14", VisionConstants.AprilTag4);
    Logger.recordOutput("Field/April Tags/April Tag 15", VisionConstants.AprilTag5);
    Logger.recordOutput("Field/April Tags/April Tag 16", VisionConstants.AprilTag6);
  }
}
