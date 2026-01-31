package frc.robot.Subsystems.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoDirector {
    // This is the chooser that will be displayed on the dashboard to select the
    // auto.
    public final SendableChooser<Auto> autoChooser = new SendableChooser<>();
    public final List<Auto> Autos = new ArrayList<>();
    private final AutoSubsystems autoSubsystems;

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
        for (Auto auto : Autos) {
            autoChooser.addOption(auto.name, auto);
        }
        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public Auto TestShooting() {
        ArrayList<Command> list = new ArrayList<>();
        list.add(new Commands.print("hello"));

        AutoTracker tracker = new AutoTracker(autoSubsystems, list, new Pose2d());

        return new Auto("hello", tracker, new Pose2d());
    }
}
