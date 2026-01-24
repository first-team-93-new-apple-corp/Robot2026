// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.NTSubsystem;
import frc.robot.Subsystems.QuestNavSubsystem;
import frc.robot.controls.*;


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
  private QuestNavSubsystem questNav = new QuestNavSubsystem(drivetrain, new Pose3d());

  public RobotContainer() {
    configureBindings();
  }

  private void updateControlScheme(String selected) {
    switch (selected) {
    case "TwoStickDrive":
      selectedControls = new TwoStickDrive(0, 1, 2);
      break;
    case "TwoStickDriveXboxOp":
      selectedControls = new TwoStickDriveXboxOp(0, 1, 2);
      break;
    case "XboxDrive":
    default:
      selectedControls = new XboxDrive(0);
      break;
    }
  }

  private void configureBindings() {
    // Controls
    controlSchemeChooser.setDefaultOption("Xbox Drive", "XboxDrive");
    controlSchemeChooser.addOption("Two Stick Drive", "TwoStickDrive");
    controlSchemeChooser.addOption("Two Stick + Xbox", "TwoStickDriveXboxOp");

    SmartDashboard.putData("Control Scheme", controlSchemeChooser);

    selectedControls = new XboxDrive(0);

    controlSchemeChooser.onChange(selected -> updateControlScheme(selected));
    drivetrain
        .setDefaultCommand(drivetrain.commands.applyRequest(() -> drive.withVelocityX(selectedControls.DriveLeft())
            .withVelocityY(selectedControls.DriveUp()).withRotationalRate(selectedControls.DriveTheta())));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
