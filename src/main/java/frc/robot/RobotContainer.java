// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ApriltagAutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavx;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public static PhotonCamera photonCamera = new PhotonCamera("limelight");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (ModeSet.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavx(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));

        // drive = new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFX(0),
        // new ModuleIOTalonFX(1),
        // new ModuleIOTalonFX(2),
        // new ModuleIOTalonFX(3));
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
    // init pathplanner
    pathplanner();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRawAxis(2)));
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .b()
        .toggleOnTrue(new ApriltagAutoAlign(photonCamera, drive, () -> drive.getPose(), 1));

    controller.back().onTrue(Commands.runOnce(drive::setEstimPoseToOdometry, drive));
  }

  private void pathplanner() {

    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser
    // built above
    SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData(
        "Pathfind to Pickup Pos",
        AutoBuilder.pathfindToPose(
            new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
            new PathConstraints(4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            0,
            2.0));
    SmartDashboard.putData(
        "Pathfind to Scoring Pos",
        AutoBuilder.pathfindToPose(
            new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
            new PathConstraints(4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            0,
            0));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m forward of its current position
    SmartDashboard.putData(
        "On-the-fly path",
        Commands.runOnce(
            () -> {
              Pose2d currentPose = drive.getPose();

              // The rotation component in these poses represents the direction of travel
              Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
              Pose2d endPos =
                  new Pose2d(
                      currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)),
                      new Rotation2d());

              List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
              PathPlannerPath path =
                  new PathPlannerPath(
                      bezierPoints,
                      new PathConstraints(
                          4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                      new GoalEndState(0.0, currentPose.getRotation()));

              AutoBuilder.followPath(path).schedule();
            }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
