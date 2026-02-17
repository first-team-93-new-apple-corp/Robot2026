package frc.robot.Subsystems.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.subsystems;

public class AutoTracker extends SequentialCommandGroup {
    private subsystems subsystems;

    public AutoTracker(subsystems subsystems, Pose2d startPose) {
        this.subsystems = subsystems;
    }

    
}
