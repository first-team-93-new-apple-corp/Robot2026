package frc.robot.Subsystems.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoTracker extends SequentialCommandGroup {
    private AutoSubsystems subsystems;

    public AutoTracker(AutoSubsystems subsystems, Supplier<Pose2d> initialPose) {
        this.subsystems = subsystems;
        addCommands(subsystems.questNav().commands.resetQuestPose(initialPose.get()));
    }
}
