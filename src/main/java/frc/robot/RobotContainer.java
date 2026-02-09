// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Set;

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
  
  //Shooter
  private ShooterMath shooterMath = new ShooterMath();

  // Auto Stuff
  private AutoSubsystems autoSubsystems = new AutoSubsystems(drivetrain, questNav, shooterMath);
  private AutoDirector auto = new AutoDirector(autoSubsystems);
  private FileWriter writer = null;
  public RobotContainer() {
    try {
      writer =  new FileWriter(new File("/U/testlog.csv"));
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
    // TODO: CHANGE ME
    // selectedControls.Seed().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).alongWith(Commands.runOnce(questNav.resetPose(drivetrain.getState().Pose)));
    controlSchemeChooser.onChange(selected -> updateControlScheme(selected));

    drivetrain
        .setDefaultCommand(drivetrain.commands.applyRequest(() -> drive.withVelocityX(selectedControls.DriveLeft() * Constants.Swerve.MaxSpeed)
            .withVelocityY(selectedControls.DriveUp() * Constants.Swerve.MaxSpeed).withRotationalRate(selectedControls.DriveTheta() * Constants.Swerve.MaxSpeed)));

    refreshBindings();


    // CommandScheduler.getInstance().schedule(questNav.commands.updateNT().ignoringDisable(true));
  }

  public void refreshBindings() {
    // selectedControls.Brake().onTrue(questNav.commands.resetQuestPose(new Pose3d()).ignoringDisable(true));
    selectedControls.Seed().onTrue(Commands.runOnce(()->questNav.resetPose(new Pose2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), new Rotation2d(0)))).andThen(drivetrain.runOnce(drivetrain::seedFieldCentric)));
  }

  public Command getAutonomousCommand() {
    return auto.selection().command();
  }
  
  public Command printPoseInfo(){
    return Commands.runOnce(()->{
      try {
        writer.write(""+ autoSubsystems.drivetrain().getState().Pose.getX() +","+autoSubsystems.drivetrain().getState().Pose.getY()+"," + autoSubsystems.questNav().getRobotPose().getX()+","+autoSubsystems.questNav().getRobotPose().getY()+"\n");
      } catch (IOException e) {
        System.out.println("Failed to write to log: "+ e);
      }
    });
  //   return ("["+ autoSubsystems.drivetrain().getState().Pose.getX() +","+autoSubsystems.drivetrain().getState().Pose.getY()+"],[" + autoSubsystems.questNav().getQuestRobotPose().getX()+autoSubsystems.questNav().getQuestRobotPose().getY()+"]");
  }

}
