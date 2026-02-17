package frc.robot.Subsystems.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.subsystems;

public class AutoTracker extends SequentialCommandGroup {
    private subsystems subsystems;

    public AutoTracker(subsystems subsystems, Pose2d startPose) {
        this.subsystems = subsystems;
    }

    public void Intake() {
        addCommands(subsystems.intake().commands.intake());
    }

    public void Outtake() {
        addCommands(subsystems.intake().commands.outtake());
    }

    public void Idle() {
        addCommands(subsystems.intake().commands.idle());
    }

    public void wigglePivot() {
        addCommands(subsystems.intake().commands.wigglePivot());
    }

    public void pivotUp() {
        addCommands(subsystems.intake().commands.autoPivotUp());
    }

    public void pivotDown() {
        addCommands(subsystems.intake().commands.autoPivotDown());
    }

    public void pivotMiddle() {
        addCommands(subsystems.intake().commands.autoPivotUp());
    }

    public void manipRollers(double speed) {
        addCommands(subsystems.manipulation().commands.intakeCommand());
    }

}
