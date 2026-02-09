package frc.robot.Subsystems.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.List;

public class AutoTracker extends SequentialCommandGroup {
    private AutoSubsystems subsystems;

    public AutoTracker(AutoSubsystems subsystems, List<Command> commandList, Supplier<Pose2d> initialPose) {
        this.subsystems = subsystems;
        // addCommands(subsystems.questNav().commands.resetQuestPose(initialPose.get()));
        for (Command command : commandList) {
            addCommands(command);
        }

    }
}
