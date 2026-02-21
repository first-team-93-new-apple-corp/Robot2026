// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.auto.*;
import frc.robot.controls.*;

public class RobotContainer {
  // Drive
  private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Swerve.MaxSpeed * Constants.Controls.Deadzone)
      .withRotationalDeadband(Constants.Swerve.MaxAngularRate * Constants.Controls.Deadzone)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(Constants.Swerve.MaxSpeed * Constants.Controls.Deadzone)
      .withRotationalDeadband(Constants.Swerve.MaxAngularRate * Constants.Controls.Deadzone) // Add a 10% deadband
      .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  // Controls
  private final SendableChooser<String> controlSchemeChooser = new SendableChooser<>();
  private ControllerSchemeIO selectedControls;

  // Network Tables
  private NTSubsystem networkTables = new NTSubsystem(new Pose2d(), new Pose2d());

  // Quest
  private QuestNavSubsystem questNav = new QuestNavSubsystem(drivetrain, new Pose3d(), networkTables);

  // Shooter
  private ShooterMath shooterMath = new ShooterMath();

  // Auto Stuff
  private AutoSubsystems autoSubsystems = new AutoSubsystems(drivetrain, questNav, shooterMath);
  private AutoDirector auto = new AutoDirector(autoSubsystems);

  public RobotContainer() {
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
    selectedControls.Seed().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).alongWith(questNav.commands.resetQuestPose(new Pose2d())));
    // selectedControls.Align().onTrue(drivetrain.alignToHub());
    driveFacingAngle.HeadingController.setPID(15, 0, 0.1);
    
    selectedControls.Align().whileTrue(drivetrain.commands.applyRequest(
      () -> driveFacingAngle.withTargetDirection(ShooterMath.generateRotation2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), drivetrain.getState().Speeds.vxMetersPerSecond,drivetrain.getState().Speeds.vyMetersPerSecond))
      .withVelocityX(selectedControls.DriveLeft() * Constants.Swerve.MaxSpeed)
      .withVelocityY(selectedControls.DriveUp() * Constants.Swerve.MaxSpeed)));

    controlSchemeChooser.onChange(selected -> updateControlScheme(selected));

    drivetrain
        .setDefaultCommand(drivetrain.commands.applyRequest(() -> drive.withVelocityX(selectedControls.DriveLeft() * Constants.Swerve.MaxSpeed)
            .withVelocityY(selectedControls.DriveUp() * Constants.Swerve.MaxSpeed).withRotationalRate(selectedControls.DriveTheta() * Constants.Swerve.MaxSpeed)));

    refreshBindings();

    
    CommandScheduler.getInstance().schedule(questNav.commands.updateNT().ignoringDisable(true));
  }

  public void refreshBindings() {
    // selectedControls.Brake().onTrue(questNav.commands.resetQuestPose(new Pose3d())).ignoringDisable(true));
    selectedControls.Brake().onTrue(questNav.commands.resetQuestPose(new Pose3d(new Pose2d(0.5,4.5,new Rotation2d()))).ignoringDisable(true));

    selectedControls.Seed().onTrue(
        questNav.commands.resetQuestPose(new Pose3d()).andThen(drivetrain.runOnce(drivetrain::seedFieldCentric)));
  }

  public Command getAutonomousCommand() {
    return Commands.defer(() -> auto.selection().command(), Set.of(drivetrain, questNav));
  }

}
