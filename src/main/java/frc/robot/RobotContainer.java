// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileWriter;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.auto.*;
import frc.robot.controls.*;
import frc.robot.util.NTSubsystem;

public class RobotContainer {
  // Drive
  private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Swerve.MaxSpeed * Constants.Controls.Deadzone)
      .withRotationalDeadband(Constants.Swerve.MaxAngularRate * Constants.Controls.Deadzone)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Controls
  private final SendableChooser<String> controlSchemeChooser = new SendableChooser<>();
  private ControllerSchemeIO selectedControls;

  // Network Tables
  private NTSubsystem networkTables = new NTSubsystem(new Pose2d(), new Pose2d());

  // Quest
  private QuestNav questNav = new QuestNav();

  // Shooter
  private ShooterMath shooterMath = new ShooterMath();

  // Auto Stuff
  private AutoSubsystems autoSubsystems = new AutoSubsystems(drivetrain, questNav, shooterMath);
  private AutoDirector auto = new AutoDirector(autoSubsystems);
  private FileWriter writer = null;

  public RobotContainer() {
    try {
      writer = new FileWriter(new File("/U/testlog.csv"));
    } catch (Exception e) {
      System.out.println("Failed to create to log");
    }
    configureBindings();
  }

  private void updateControlScheme(String selected) {
    if (selected.equals("TwoStickDrive")) {
      selectedControls = new TwoStickDrive(0, 1);
    } else if (selected.equals("TwoStickDriveXboxOp")) {
      selectedControls = new TwoStickDriveXboxOp(0, 1, 2);
    } else {
      selectedControls = new XboxDrive(0);
    }
    refreshBindings();
  }

  private void configureBindings() {

    // Controls
    controlSchemeChooser.addOption("Xbox Drive", "XboxDrive");
    controlSchemeChooser.setDefaultOption("Two Stick Drive", "TwoStickDrive");
    controlSchemeChooser.addOption("Two Stick + Xbox", "TwoStickDriveXboxOp");

    SmartDashboard.putData("Control Scheme", controlSchemeChooser);

    selectedControls = new TwoStickDrive(0, 1);

    selectedControls.Seed().onTrue(seed());

    controlSchemeChooser.onChange(selected -> updateControlScheme(selected));

    drivetrain
        .setDefaultCommand(drivetrain.commands
            .applyRequest(() -> drive.withVelocityX(selectedControls.DriveLeft() * Constants.Swerve.MaxSpeed)
                .withVelocityY(selectedControls.DriveUp() * Constants.Swerve.MaxSpeed)
                .withRotationalRate(selectedControls.DriveTheta() * Constants.Swerve.MaxSpeed)));

    refreshBindings();
  }

  public void refreshBindings() {
    selectedControls.Seed().onTrue(seed());
  }

  public Command getAutonomousCommand() {
    return auto.selection().command();
  }
  

  /** 
   * Periodic method to run things that need periodic. Set to 20ms with a 5ms offset
   */
  public void visionPeriodic() {
    questNav.updateVisionMeasurement(drivetrain);
  }
  
  /***
   * Seeds the drivetrain and questnav.
   * 
   * @return a command that seeds the drivetrain and questnav
   */
  public Command seed() {
    return Commands.runOnce(() -> {
      drivetrain.seedFieldCentric();
      // questNav.resetPose2D(questNav.getQuestPose2D().rotateBy(questNav.getQuestPose2D().getRotation().unaryMinus()));
    });
  }
/**
 * Prints the drivetrain and questnav pose to a log file. This is used for offline analysis of the pose estimation performance.
 * The log file will have the following format:
 * timestamp, drivetrainX, drivetrainY, questnav2DX, questnav2DY, questnav3DX, questnav3DY, questnav3DZ
 * @return Command to run the logging action
 */
  public Command printPoseInfo() {
    return Commands.runOnce(() -> {
      try {
        double[] drivetrainPose = {
            autoSubsystems.drivetrain().getState().Pose.getX(),
            autoSubsystems.drivetrain().getState().Pose.getY() };
        double[] questPose2D = {
            autoSubsystems.questNav().getAverageRobotPose2D().getX(),
            autoSubsystems.questNav().getAverageRobotPose2D().getY() };
        double[] questPose3D = {
            autoSubsystems.questNav().getAverageRobotPose3D().getX(),
            autoSubsystems.questNav().getAverageRobotPose3D().getY(),
            autoSubsystems.questNav().getAverageRobotPose3D().getZ() };
        double timestamp = autoSubsystems.questNav().getTimestamp();
        String logEntry = String.format(
            "%f,%f,%f,%f,%f,%f,%f\n",
            timestamp,
            drivetrainPose[0],
            drivetrainPose[1],
            questPose2D[0],
            questPose2D[1],
            questPose3D[0],
            questPose3D[1],
            questPose3D[2]);
        writer.write(logEntry);
      } catch (Exception e) {
        System.out.println("Failed to write to log: " + e);
      }
    });
  }

}