package frc.robot.Subsystems.auto;

import static edu.wpi.first.units.Units.Seconds;

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
import frc.robot.Constants.ShooterConstants.Presets;
import frc.robot.util.NTSubsystem;
import frc.robot.util.subsystems;

public class AutoDirector {
    // This is the chooser that will be displayed on the dashboard to select the
    // auto.
    public final SendableChooser<Auto> autoChooser = new SendableChooser<>();
    public final List<Auto> Autos = new ArrayList<>();
    private final subsystems autoSubsystems;
    private final NTSubsystem networkTables;
    private static RobotConfig config = null;
    private String previewedAutoName = "";

    // Auto
    private SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public AutoDirector(subsystems autoSubsystems, NTSubsystem networkTables) {
        this.autoSubsystems = autoSubsystems;
        this.networkTables = networkTables;
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
        autoSubsystems.vision().commands.setRobotPose(new Pose3d(pose));
    }

    public record Auto(String name, Command command, Pose2d initPose, List<Pose2d> previewPoses,
            List<Pose2d> previewWaypoints) {
        public Auto(String name, Command command, Pose2d initPose) {
            this(name, command, initPose, List.of(), List.of());
        }

        public Auto(String name, Command command) {
            this(name, command, new Pose2d(), List.of(), List.of());
        }
    }

    public Auto selection() {
        return autoChooser.getSelected();
    }

    public boolean hasSelectedAutoPreviewChanged() {
        Auto selectedAuto = selection();
        String selectedName = selectedAuto == null ? "" : selectedAuto.name;
        return !selectedName.equals(previewedAutoName);
    }

    public void updateSelectedAutoPreview() {
        Auto selectedAuto = selection();
        if (selectedAuto == null) {
            if (!previewedAutoName.isEmpty()) {
                networkTables.quest.clearAutoPreview();
                previewedAutoName = "";
                return;
            }
            return;
        }

        // if (selectedAuto.name.equals(previewedAutoName)) {
        //     return;
        // }

        networkTables.quest.updateAutoPreview(selectedAuto.name, selectedAuto.previewPoses, selectedAuto.previewWaypoints);
        previewedAutoName = selectedAuto.name;
    }

    public void removePreview(){
        networkTables.quest.clearAutoPreview();
    }

    public void addAutos() {
        autoChooser.setDefaultOption(Default().name, Default());

        Autos.add(DoNothing());
        Autos.add(PreloadClose());
        Autos.add(PreloadCloseLeft());
        Autos.add(PreloadCloseRight());
        Autos.add(PreloadLeftBump());
        Autos.add(PreloadRightBump());
        Autos.add(DepotScoreClose());
        Autos.add(DepotScoreCloseLeft());
        Autos.add(DepotScoreCloseRight());
        Autos.add(DepotScoreLeftBump());
        Autos.add(DepotScoreRightBump());
        Autos.add(OutpostScoreClose());
        Autos.add(OutpostScoreCloseLeft());
        Autos.add(OutpostScoreCloseRight());
        Autos.add(OutpostScoreLeftBump());
        Autos.add(OutpostScoreRightBump());
        Autos.add(LeftCenterLeftHalfScoreClose());
        Autos.add(LeftCenterLeftHalfScoreCloseLeft());
        Autos.add(LeftCenterLeftHalfScoreCloseRight());
        Autos.add(LeftCenterLeftHalfScoreBump());
        Autos.add(RightCenterRightHalfScoreClose());
        Autos.add(RightCenterRightHalfScoreCloseLeft());
        Autos.add(RightCenterRightHalfScoreCloseRight());
        Autos.add(RightCenterRightHalfScoreBump());
        Autos.add(RightDoubleDip());
        Autos.add(RightDoubleDip2());
        Autos.add(RightCenterThenOutpost());
        Autos.add(LeftDoubleDip());
        Autos.add(LeftDoubleDip2());
        Autos.add(LeftCenterThenDepot());
        Autos.add(LeftToRightFull());
        Autos.add(RightToLeftFull());
        Autos.add(LeftCenterLeftHalfScoreBump());




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
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addShootPath("Shoot Center", Presets.close);
        tracker.endAuto();
        return trackedAuto("Pre Close", tracker);
    }

    public Auto PreloadCloseLeft() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addShootPath("Shoot Center Left", Presets.closeSide);
        tracker.endAuto();
        return trackedAuto("Pre Close Left", tracker);
    }

    public Auto PreloadCloseRight() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addShootPath("Shoot Center Right", Presets.closeSide);
        tracker.endAuto();
        return trackedAuto("Pre Close Right", tracker);
    }

    public Auto PreloadLeftBump() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addShootPath("Left Bump Shoot", Presets.bump);
        tracker.endAuto();
        return trackedAuto("Pre L Bump", tracker);
    }

    public Auto PreloadRightBump() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addShootPath("Right Bump Shoot", Presets.bump);
        tracker.endAuto();
        return trackedAuto("Pre R Bump", tracker);
    }

    public Auto DepotScoreClose() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addIntakePath("depotIntake");
        tracker.addShootPath("Shoot Center", Presets.close);
        tracker.endAuto();
        return trackedAuto("D Close", tracker);
    }

    public Auto DepotScoreCloseLeft() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addIntakePath("depotIntake");
        tracker.addShootPath("Shoot Center Left", Presets.closeSide);
        tracker.endAuto();
        return trackedAuto("D Close Left", tracker);
    }

    public Auto DepotScoreCloseRight() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addIntakePath("depotIntake");
        tracker.addShootPath("Shoot Center Right", Presets.closeSide);
        tracker.endAuto();
        return trackedAuto("D Close Right", tracker);
    }

    public Auto DepotScoreRightBump() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addIntakePath("depotIntake");
        tracker.addShootPath("Right Bump Shoot", Presets.bump);
        tracker.endAuto();
        return trackedAuto("D Right Bump", tracker);
    }
    public Auto DepotScoreLeftBump() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addIntakePath("depotIntake");
        tracker.addShootPath("Left Bump Shoot", Presets.bump);
        tracker.endAuto();
        return trackedAuto("D Left Bump", tracker);
    }

    public Auto OutpostScoreClose() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addIntakePath("outpostIntake");
        tracker.addCommands(Commands.waitSeconds(5));
        tracker.addShootPath("Shoot Center", Presets.close);
        tracker.endAuto();
        return trackedAuto("O Close", tracker);
    }

    public Auto OutpostScoreCloseLeft() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addIntakePath("outpostIntake");
        tracker.addCommands(Commands.waitSeconds(5));
        tracker.addShootPath("Shoot Center Left", Presets.closeSide);
        tracker.endAuto();
        return trackedAuto("O Close Left", tracker);
    }

    public Auto OutpostScoreCloseRight() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addIntakePath("outpostIntake");
        tracker.addCommands(Commands.waitSeconds(5));
        tracker.addShootPath("Shoot Center Right", Presets.closeSide);
        tracker.endAuto();
        return trackedAuto("O Close Right", tracker);
    }

    public Auto OutpostScoreRightBump() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addIntakePath("outpostIntake");
        tracker.addCommands(Commands.waitSeconds(5));
        tracker.addShootPath("Right Bump Shoot", Presets.bump);
        tracker.endAuto();
        return trackedAuto("O Right Bump", tracker);
    }

    public Auto OutpostScoreLeftBump() {
        
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addIntakePath("outpostIntake");
        tracker.addCommands(Commands.waitSeconds(5));
        tracker.addShootPath("Left Bump Shoot", Presets.bump);
        tracker.endAuto();
        return trackedAuto("O Left Bump", tracker);
    }

    public Auto LeftCenterLeftHalfScoreClose() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("Overbump");
        tracker.addIntakePath("L_Center_Intake");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBump("BackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Shoot Center", Presets.close);
        tracker.endAuto();
        return trackedAuto("L Center Left CloseV2", tracker);
    }

    public Auto LeftCenterLeftHalfScoreCloseLeft() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("Overbump");
        tracker.addIntakePath("L_Center_Intake");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBump("BackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Shoot Center Left", Presets.closeSide);
        tracker.endAuto();
        return trackedAuto("L Center Close Left", tracker);
    }

    public Auto LeftCenterLeftHalfScoreCloseRight() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("Overbump");
        tracker.addIntakePath("L_Center_Intake");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBump("BackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Shoot Center Right", Presets.closeSide);
        tracker.endAuto();
        return trackedAuto("L Center Close Right", tracker);
    }

     public Auto LeftCenterLeftHalfScoreBump() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("Overbump");
        tracker.addIntakePath("L_Center_Intake");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBump("BackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Left Bump Shoot", Presets.bump, Seconds.of(14));
        tracker.endAuto();
        return trackedAuto("L Center Bump", tracker);
    }


    public Auto RightCenterRightHalfScoreClose() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("RightOverbump");
        tracker.addIntakePath("R_Center_Intake");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBump("RightBackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Shoot Center", Presets.close);
        tracker.endAuto();
        return trackedAuto("R Center Close", tracker);
    }

    public Auto RightCenterRightHalfScoreCloseLeft() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("RightOverbump");
        tracker.addIntakePath("R_Center_Intake");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBump("RightBackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Shoot Center Left", Presets.closeSide);
        tracker.endAuto();
        return trackedAuto("R Center Left Close", tracker);
    }

    public Auto RightCenterRightHalfScoreCloseRight() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("RightOverbump");
        tracker.addIntakePath("R_Center_Intake");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBump("RightBackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Shoot Center Right", Presets.closeSide);
        tracker.endAuto();
        return trackedAuto("R Center Right Close", tracker);
    }

    public Auto RightCenterRightHalfScoreBump() {
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("RightOverbump");
        tracker.addIntakePath("R_Center_Intake");
        // tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBump("RightBackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Right Bump Shoot", Presets.bump);
        tracker.endAuto();
        return trackedAuto("R Center Bump", tracker);
    }

    public Auto RightDoubleDip(){
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("RightOverbump");
        tracker.addIntakePath("R_Center_Intake");
        tracker.addOverBump("RightBackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Right Bump Shoot", Presets.bump, Seconds.of(4));
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addOverBump("RightOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addIntakePath("R_Center_Intake_Behind_Hub");
        tracker.addOverBump("RightBackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Right Bump Shoot", Presets.bump, Seconds.of(4));
        tracker.endAuto();
        return trackedAuto("R Double Center-Behind Hub", tracker);
    }

    public Auto RightDoubleDip2(){
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("RightOverbump");
        tracker.addIntakePath("R_Center_Intake");
        tracker.addOverBump("RightBackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Right Bump Shoot", Presets.bump, Seconds.of(4));
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addOverBump("RightOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addIntakePath("R_Center_Intake_22");
        tracker.addOverBump("RightBackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Right Bump Shoot", Presets.bump, Seconds.of(4));
        tracker.endAuto();
        return trackedAuto("R Double Center-Center", tracker);
    }

    public Auto RightCenterThenOutpost(){
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("RightOverbump");
        tracker.addIntakePath("R_Center_Intake");
        tracker.addOverBump("RightBackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Right Bump Shoot", Presets.bump, Seconds.of(4));
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addIntakePath("outpostIntake");
        tracker.addCommands(Commands.waitSeconds(4));
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Right Bump Shoot", Presets.bump, Seconds.of(4));
        tracker.endAuto();
        return trackedAuto("R Double Center-Outpost", tracker);
    }

    public Auto LeftDoubleDip(){
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("Overbump");
        tracker.addIntakePath("L_Center_Intake");
        tracker.addOverBump("BackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Left Bump Shoot", Presets.bump, Seconds.of(4));
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addOverBump("Overbump");
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addIntakePath("L_Center_Intake_Behind_Hub");
        tracker.addOverBump("BackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Left Bump Shoot", Presets.bump, Seconds.of(4));
        tracker.endAuto();
        return trackedAuto("L Double Center-Behind Hub", tracker);
    }

    public Auto LeftDoubleDip2(){
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("Overbump");
        tracker.addIntakePath("L_Center_Intake");
        tracker.addOverBump("BackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Left Bump Shoot", Presets.bump, Seconds.of(4));
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addOverBump("Overbump");
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addIntakePath("L_Center_Intake_2");
        tracker.addOverBump("BackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Left Bump Shoot", Presets.bump, Seconds.of(4));
        tracker.endAuto();
        return trackedAuto("L Double Center", tracker);
    }

    public Auto LeftCenterThenDepot(){
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("Overbump");
        tracker.addIntakePath("L_Center_Intake");
        tracker.addOverBump("BackOverbump");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Left Bump Shoot", Presets.bump, Seconds.of(4));
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addIntakePath("depotIntake");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addShootPath("Left Bump Shoot", Presets.bump, Seconds.of(4));
        tracker.endAuto();
        return trackedAuto("L Double Center-Depot", tracker);
    }

    public Auto LeftToRightFull(){
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("Overbump");
        tracker.addIntakePath("L_R_Full");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBump("RightBackOverbump");
        tracker.addShootPath("Right Bump Shoot", Presets.bump, Seconds.of(8));
        tracker.endAuto();
        tracker.endAuto();
        return trackedAuto("L To R Full Center", tracker);
    }

    public Auto RightToLeftFull(){
        AutoTracker tracker = new AutoTracker(autoSubsystems);
        tracker.addCommands(autoSubsystems.intake().commands.autoPivotDown());
        tracker.addCommands(autoSubsystems.shooter().commands.velocityAndHood(()->Presets.bump));
        tracker.addOverBump("RightOverbump");
        tracker.addIntakePath("R_L_Full");
        tracker.addCommands(autoSubsystems.intake().commands.stop());
        tracker.addOverBump("BackOverbump");
        tracker.addShootPath("Left Bump Shoot", Presets.bump, Seconds.of(8));
        tracker.endAuto();
        return trackedAuto("R To L Full Center", tracker);
    }

    private Auto trackedAuto(String name, AutoTracker tracker) {
        return new Auto(name, tracker, new Pose2d(), tracker.getPreviewPoses(), tracker.getPreviewWaypoints());
    }
}
