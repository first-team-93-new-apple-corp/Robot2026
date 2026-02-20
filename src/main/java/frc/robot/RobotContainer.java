// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.auto.*;
import frc.robot.Controls.*;
import frc.robot.util.NTSubsystem;
import frc.robot.util.subsystems;

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

    // Vision
    // * Quest
    private QuestNavSubsystem questNav = new QuestNavSubsystem(drivetrain, networkTables);

    // Subsystems
    // * Shooter
    private ShooterMath shooterMath = new ShooterMath();
    private ClimberSubsystem climber = new ClimberSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ManipulationSubsystem manipulation = new ManipulationSubsystem();

    // subsytems var, contains all subsytems, less to implemnt into classes
    // private subsystems subsystems = new subsystems(drivetrain, questNav,
    // shooterMath, climber, intake, manipulation);
    private subsystems subsystems = new subsystems(drivetrain, questNav, shooterMath, climber, intake, manipulation);
    private AutoDirector auto = new AutoDirector(subsystems);
    private FileWriter writer = null;

    public RobotContainer() {
        //
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
                        .applyRequest(() -> drive
                                .withVelocityX(selectedControls.DriveLeft() * Constants.Swerve.MaxSpeed)
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
     * Periodic method to run things that need periodic. Set to 20ms with a 5ms
     * offset
     */
    public void visionPeriodic() {
        questNav.visionPeriodic();
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
     * Prints the drivetrain and questnav pose to a log file. This is used for
     * offline analysis of the pose estimation performance.
     * The log file will have the following format:
     * timestamp, drivetrainX, drivetrainY, questnav2DX, questnav2DY, questnav3DX,
     * questnav3DY, questnav3DZ
     * 
     * @return Command to run the logging action
     */
    public Command printPoseInfo() {
        return Commands.runOnce(() -> {
            try {
                writer.write(subsystems.questNav().questPoseInfo() + "\n");
            } catch (IOException e) {
                System.out.println("Failed to write to log: " + e.getMessage());
            }
        });
    }

}