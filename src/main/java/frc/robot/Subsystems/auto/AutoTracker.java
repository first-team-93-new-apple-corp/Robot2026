package frc.robot.Subsystems.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.List;
import java.util.ArrayList;

public class AutoTracker extends SequentialCommandGroup {
    private AutoSubsystems subsystems;

    public AutoTracker(AutoSubsystems subsystems, ArrayList<Command> commandList, Supplier<Pose2d> initialPose) {
        this.subsystems = subsystems;
        addCommands(subsystems.questNav().commands.resetQuestPose(initialPose.get()));
        
        }
    
}
