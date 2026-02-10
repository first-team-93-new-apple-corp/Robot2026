package frc.robot.Subsystems.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;

public class AutoDirector {
    // This is the chooser that will be displayed on the dashboard to select the
    // auto.
    public final SendableChooser<Auto> autoChooser = new SendableChooser<>();
    public final List<Auto> Autos = new ArrayList<>();
    private final AutoSubsystems autoSubsystems;
    private final PathConstraints constraints = Constants.Auto.pathConstraints;

    public AutoDirector(AutoSubsystems autoSubsystems) {
        this.autoSubsystems = autoSubsystems;
        addAutos();
    }

    // This is the basis of the Auto. It contains the name of the auto, the command
    // to run, and the initial pose.
    public record Auto(String name, Command command, Pose2d initPose) {
        // This allows us to create an Auto without specifying an initial pose,
        // defaulting to field origin.
        public Auto(String name, Command command) {
            this(name, command, new Pose2d());
        }
    }

    public Auto selection() {
        return autoChooser.getSelected();
    }

    public void addAutos() {
        autoChooser.setDefaultOption("Do Nothing", new Auto("Do Nothing", Commands.none()));
        // Autos.add([Auto]);
        Autos.add(TestShooting());
        Autos.add(TestAuto());
        for (Auto auto : Autos) {
            autoChooser.addOption(auto.name, auto);
        }
        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public Auto TestShooting() {
        List<Command> list = new ArrayList<>();
        list.add(Commands.print("Testing Shooting Math"));
        Pose2d currentPose = autoSubsystems.drivetrain().getState().Pose;
        double hubX = (Math
                .sqrt(Math.pow(4.625594 - currentPose.getX(), 2) + Math.pow(4.034536 - currentPose.getY(), 2)));
        double hubY = 1.82;
        double angle = autoSubsystems.shooterMath().calculateAngle(0.0, 0.0, hubX - 0.5, hubY + 0.5, hubX, hubY);

        list.add(Commands.print("Shoot at angle " + angle + "Shoot at velocity "
                + autoSubsystems.shooterMath().calculateV(angle, hubX, hubY, -9.8)));

        AutoTracker tracker = new AutoTracker(autoSubsystems, list, () -> new Pose2d());

        return new Auto("TestShooting", tracker, new Pose2d());
    }

    public Auto TestAuto() {
        PathPlannerPath testPath = null;
        List<Command> list = new ArrayList<>();

        try {
            testPath = PathPlannerPath.fromPathFile("Many Over Bump");
        } catch (Exception e) {
            e.printStackTrace();
        }
        Pose2d startPose = testPath.getStartingDifferentialPose();
        Pose2d correctedStartPose = new Pose2d(startPose.getX(), startPose.getY(), new Rotation2d());
        // Shhh definintly not doing this vvv
        // list.add(autoSubsystems.questNav().commands.resetQuestPose(correctedStartPose));
        // Shhh definintly not doing this ^^^^
        list.add(Commands.print("Testing Auto"));
        list.add(Commands.print("****************************************** START POSE: " + startPose.toString()));
        list.add(Commands.print("****************************************** Better POSE: " + correctedStartPose.toString()));

        list.add(AutoBuilder.followPath(testPath));
        Command cmd = AutoBuilder.pathfindToPose(correctedStartPose, constraints).andThen(Commands.print("Pathfound back to start: "+ correctedStartPose.toString()));
        list.add(cmd);

        AutoTracker tracker = new AutoTracker(autoSubsystems, list, () -> correctedStartPose);

        return new Auto("TestAuto", tracker, correctedStartPose);
    }
}
