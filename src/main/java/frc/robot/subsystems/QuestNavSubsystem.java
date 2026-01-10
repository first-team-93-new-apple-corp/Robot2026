package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    private QuestNav quest;
    private boolean poseSet = false;

    // http://10.0.93.200:5801/

    public QuestCommands commands = new QuestCommands();

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain, Pose3d startingPose) {
        this.drivetrain = drivetrain;
        quest = new QuestNav();

        // Assume this is the requested reset pose
        Pose3d robotPose = startingPose;

        // Transform by the offset to get the Quest pose
        Pose3d questPose = robotPose.transformBy(Constants.Quest.RobotToQuest);

        // Send the reset operation
        quest.setPose(questPose);
    }

    public void setPose(Pose3d newRobotPose) {
        Pose3d questPose = newRobotPose.transformBy(Constants.Quest.RobotToQuest);
        quest.setPose(questPose);
        poseSet = true;

    }

    @Override
    public void periodic() {
        quest.commandPeriodic();

        SmartDashboard.putBoolean("Quest Connected", quest.isConnected());
        SmartDashboard.putNumber("Quest Battery %", quest.getBatteryPercent().getAsInt());

        if (quest.isTracking()) {
            // Get the latest pose data frames from the Quest
            PoseFrame[] questFrames = quest.getAllUnreadPoseFrames();

            // Loop over the pose data frames and send them to the pose estimator
            for (PoseFrame questFrame : questFrames) {
                // Get the pose of the Quest
                Pose3d questPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose
                Pose3d robotPose = questPose.transformBy(Constants.Quest.RobotToQuest.inverse());

                // Add the measurement to our estimator
                drivetrain.addVisionMeasurement(robotPose.toPose2d(), timestamp, Constants.Quest.QUESTNAV_STD_DEVS);
            }
        }
    }

    public class QuestCommands {
        public Command resetQuestPose(Pose3d newRobotPose) {
            return Commands.runOnce(() -> {
                Pose3d questPose = newRobotPose.transformBy(Constants.Quest.RobotToQuest);
                quest.setPose(questPose);
            });
        }

        public Command resetQuestPose(Pose2d newRobotPose) {
            return Commands.runOnce(() -> {
                Pose3d newRobotPose3d = new Pose3d(newRobotPose.getX(), newRobotPose.getY(), 0,
                        new Rotation3d(newRobotPose.getRotation().getDegrees(), 0, 0));
                Pose3d questPose = newRobotPose3d.transformBy(Constants.Quest.RobotToQuest);
                quest.setPose(questPose);
            });
        }
    }
}
