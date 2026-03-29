package frc.robot.Subsystems.auto;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.subsystems;

public class AutoDirector {
    // This is the chooser that will be displayed on the dashboard to select the
    // auto.
    public final SendableChooser<Auto> autoChooser = new SendableChooser<>();
    public final List<Auto> Autos = new ArrayList<>();
    private final subsystems autoSubsystems;
    private static RobotConfig config = null;

    // Auto
    private SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public AutoDirector(subsystems autoSubsystems) {
        this.autoSubsystems = autoSubsystems;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        AutoBuilder.configure(
                this::getPose,
                this::resetAutoPose,
                this::getSpeeds,
                (speeds, feedforwards) -> autoSubsystems.drivetrain().setControl(autoRequest.withSpeeds(speeds)),
                new PPHolonomicDriveController(
                        new PIDConstants(9.5, 0.0, 0.001),
                        new PIDConstants(4.5, 0.0, 0.0)),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                autoSubsystems.drivetrain());
        addAutos();

    }

    public static RobotConfig getRobotConfig() {
        return config;
    }

    private Pose2d getPose() {
        return autoSubsystems.drivetrain().getState().Pose;
    }

    private ChassisSpeeds getSpeeds() { 
        return autoSubsystems.drivetrain().getState().Speeds;
    }

    public void resetAutoPose(Pose2d pose) {
        autoSubsystems.drivetrain().resetPose(pose);
        autoSubsystems.questNav().commands.setRobotPose(new Pose3d(pose));
    }

    public record Auto(String name, Command command, Pose2d initPose) {
        public Auto(String name, Command command) {
            this(name, command, new Pose2d());
        }
    }

    public Auto selection() {
        return autoChooser.getSelected();
    }

    public void addAutos() {
        autoChooser.setDefaultOption(Default().name, Default());
        // try {
        //     Autos.add(TuningAuto());
        //     Autos.add(TuningAuto2());
        //     Autos.add(TuningAuto3());
        // } catch (Exception e) {
        //     // TODO: handle exception
        // }
        Autos.add(DoNothing());
        // *** vvv Confident
        Autos.add(PreloadClose());
        Autos.add(PreloadCloseLeft());
        Autos.add(PreloadCloseRight());
        Autos.add(DepotScoreClose());
        Autos.add(DepotScoreCloseLeft());
        Autos.add(DepotScoreCloseRight());
        // *** ^^^ Confident
        // *** vvv Never tested
        Autos.add(OutpostScoreClose());
        // *** ^^^ never tested
        // *** vvvvv maybe?!?!? might not finish in time
        Autos.add(LeftCenterLeftHalfScoreClose());
        Autos.add(LeftCenterLeftHalfScoreCloseLeft());
        Autos.add(LeftCenterLeftHalfScoreCloseRight());
        Autos.add(RightCenterRightHalfScoreClose());
        Autos.add(RightCenterRightHalfScoreCloseLeft());
        Autos.add(RightCenterRightHalfScoreCloseRight());
        // *** ^^^^^  maybe?!?!? might not finish in time



        for (Auto auto : Autos) {
            autoChooser.addOption(auto.name, auto);
        }
        SmartDashboard.putData("AutoChooser", autoChooser);
    }
    public Auto TuningAuto() throws IOException, FileVersionException, org.json.simple.parser.ParseException {
        return new Auto("TuningTranslational", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Tuning Path Forward")));
    }

    public Auto TuningAuto2() throws IOException, FileVersionException, org.json.simple.parser.ParseException {
        return new Auto("TuningTranslationalInverse", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Tuning Path Backward")));
    }

    public Auto TuningAuto3() throws IOException, FileVersionException, org.json.simple.parser.ParseException {
        return new Auto("TuningTranslationalRotate", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Tuning Path 90")));
    }


    public Auto combineAutos(Auto... autos) {
        List<Command> list = new ArrayList<>();
        for (Auto auto : autos) {
            list.add(Commands.print("Starting Auto: " + auto.name));
            list.add(auto.command);
        }
        SequentialCommandGroup cmds = new SequentialCommandGroup();
        for (Command command : list) {
            cmds.addCommands(command);
        }
        return new Auto("Combined Auto", cmds, new Pose2d());
    }

    public Alliance getAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get();
        }
        return Alliance.Blue;
    }

    public Auto DoNothing(){
        return new Auto("Do Nothing", Commands.none());
    }

    public Auto Default() {
        return PreloadClose();
    }

    public Auto PreloadClose() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addShootPath("Shoot Center", AutoConstants.PresetShootingPoints.getClose());
        tracker.endAuto();
        return new Auto("Score Preload Only Close", tracker);
    }

    public Auto PreloadCloseLeft() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addShootPath("Shoot Center Left", AutoConstants.PresetShootingPoints.getDepotLeft());
        tracker.endAuto();
        return new Auto("Score Preload Only Close Left", tracker);
    }

    public Auto PreloadCloseRight() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addShootPath("Shoot Center Right", AutoConstants.PresetShootingPoints.getClose());
        tracker.endAuto();
        return new Auto("Score Preload Only Close Right", tracker);
    }

    public Auto DepotScoreClose() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addIntakePath("depotIntake");
        tracker.addShootPath("Shoot Center", AutoConstants.PresetShootingPoints.getClose());
        tracker.endAuto();
        return new Auto("Depot Shoot Center", tracker);
    }

    public Auto DepotScoreCloseLeft() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addIntakePath("depotIntake");
        tracker.addShootPath("Shoot Center Left", AutoConstants.PresetShootingPoints.getCloseSide());
        tracker.endAuto();
        return new Auto("Depot Shoot Center Left", tracker);
    }

    public Auto DepotScoreCloseRight() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addIntakePath("depotIntake");
        tracker.addShootPath("Shoot Center Right", AutoConstants.PresetShootingPoints.getCloseSide());
        tracker.endAuto();
        return new Auto("Depot Score Center Right", tracker);
    }

    public Auto OutpostScoreClose() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addIntakePath("depotIntake");
        tracker.addShootPath("Shoot Center", AutoConstants.PresetShootingPoints.getClose());
        tracker.endAuto();
        return new Auto("Outpost Score Center", tracker);
    }

    public Auto OutpostScoreCloseLeft() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addIntakePath("depotIntake");
        tracker.addShootPath("Shoot Center Left", AutoConstants.PresetShootingPoints.getCloseSide());
        tracker.endAuto();
        return new Auto("Outpost Score Center Left", tracker);
    }

    public Auto OutpostScoreCloseRight() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addIntakePath("depotIntake");
        tracker.addShootPath("Shoot Center Right", AutoConstants.PresetShootingPoints.getCloseSide());
        tracker.endAuto();
        return new Auto("Outpost Score Center Right", tracker);
    }

    public Auto LeftCenterLeftHalfScoreClose() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addOverBumpLeft("Overbump");
        tracker.addIntakePath("L_Center_Intake");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBumpLeft("BackOverbump");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Shoot Center", AutoConstants.PresetShootingPoints.getClose());
        tracker.endAuto();
        return new Auto("Left Side Intake Left Half Center Score Close", tracker);
    }

    public Auto LeftCenterLeftHalfScoreCloseLeft() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addOverBumpLeft("Overbump");
        tracker.addIntakePath("L_Center_Intake");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBumpLeft("BackOverbump");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Shoot Center Left", AutoConstants.PresetShootingPoints.getCloseSide());
        tracker.endAuto();
        return new Auto("Left Side Intake Center Score Close Left", tracker);
    }

    public Auto LeftCenterLeftHalfScoreCloseRight() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addOverBumpLeft("Overbump");
        tracker.addIntakePath("L_Center_Intake");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBumpLeft("BackOverbump");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Shoot Center Left", AutoConstants.PresetShootingPoints.getCloseSide());
        tracker.endAuto();
        return new Auto("Left Side Intake Center Score Close Right", tracker);
    }


    public Auto RightCenterRightHalfScoreClose() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addOverBumpLeft("RightOverbump");
        tracker.addIntakePath("R_Center_Intake");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBumpLeft("RightBackOverbump");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Shoot Center", AutoConstants.PresetShootingPoints.getClose());
        tracker.endAuto();
        return new Auto("Right Side Intake Center Score Close", tracker);
    }

    public Auto RightCenterRightHalfScoreCloseLeft() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addOverBumpLeft("RightOverbump");
        tracker.addIntakePath("L_Center_Intake");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBumpLeft("RightBackOverbump");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Shoot Center Left", AutoConstants.PresetShootingPoints.getCloseSide());
        tracker.endAuto();
        return new Auto("Right Side Intake Center Score Close Left", tracker);
    }

    public Auto RightCenterRightHalfScoreCloseRight() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addOverBumpLeft("RightOverbump");
        tracker.addIntakePath("R_Center_Intake");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBumpLeft("RightBackOverbump");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Shoot Center Right", AutoConstants.PresetShootingPoints.getCloseSide());
        tracker.endAuto();
        return new Auto("Right Side Intake Center Score Close Right", tracker);
    }
}
