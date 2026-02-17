package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

// import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.util.NTSubsystem;
import frc.robot.util.RollingAveragePose3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem {
    private CommandSwerveDrivetrain drivetrain;
    private QuestNav quest;

    // http://10.0.93.200:5801/

    public QuestCommands commands = new QuestCommands();
    private NTSubsystem networkTables;

    // private Pose2d questPose2d = new Pose2d();
    private Pose3d questPose3d = new Pose3d();

    private Pose2d robotPose2d = new Pose2d();
    private Pose3d robotPose3d = new Pose3d();

    private RollingAveragePose3d robotPoseAverager = new RollingAveragePose3d(10);
    private RollingAveragePose3d questPoseAverager = new RollingAveragePose3d(10);

    // private Pose3d piPose3d = new Pose3d();

    // private PhotonCamera camera = new PhotonCamera("MainCam");
    // private boolean hasPoseInit = false;
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCam = new Transform3d(
            new Translation3d(Inches.of(13), Inches.of(13.25), Inches.of(12)),
            new Rotation3d(Degrees.of(0), Degrees.of(22.5), Degrees.of(0)));
    // private PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain, NTSubsystem nt) {
        this.drivetrain = drivetrain;
        this.networkTables = nt;
        quest = new QuestNav();

        // Transform by the offset to get the Quest pose
        questPose3d = robotPose3d.transformBy(Constants.Quest.RobotToQuest3D);

        // Send the reset operation
        quest.setPose(questPose3d);

        // this.networkTables = networkTables;

        quest.setVersionCheckEnabled(false);

        robotPoseAverager.addPose(robotPose3d);
        questPoseAverager.addPose(questPose3d);
    }

    public void setPose(Pose3d newRobotPose) {
        Pose3d questPose = newRobotPose.transformBy(Constants.Quest.RobotToQuest3D);
        quest.setPose(questPose);
    }

    /**
     * This is the method that should be called periodically to update quest
     * measurements
     */
    public void visionPeriodic() {
        quest.commandPeriodic();

        SmartDashboard.putBoolean("Quest Connected", quest.isConnected());
        SmartDashboard.putBoolean("Quest Tracking?", quest.isTracking());
        SmartDashboard.putNumber("Quest Battery %", quest.getBatteryPercent().getAsInt());
        SmartDashboard.putNumber("Quest Tracking Lost", quest.getTrackingLostCounter().getAsInt());

        if (quest.isTracking()) {
            // Get the latest pose data frames from the Quest
            PoseFrame[] questFrames = quest.getAllUnreadPoseFrames();

            // Loop over the pose data frames and send them to the pose estimator
            for (PoseFrame questFrame : questFrames) {
                // Get the pose of the Quest
                questPose3d = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                // Transform by the mount pose to get your robot pose
                robotPose3d = questPose3d.transformBy(Constants.Quest.RobotToQuest3D.inverse());
                robotPose2d = robotPose3d.toPose2d();

                // Add the measurement to our estimator
                addVisionMeasurement(drivetrain, robotPose3d);

            }
        }

        networkTables.quest.updateQuestPose(questPose3d);
        networkTables.quest.updateRobotPose(robotPose3d);

        networkTables.quest.updateAvgQuestPose(getAverageQuestPose3D());
        networkTables.quest.updateAvgRobotPose(getAverageRobotPose3D());

        questPoseAverager.addPose(questPose3d);
        robotPoseAverager.addPose(robotPose3d);
    }

    public String questPoseInfo() {
        double timestamp = RobotController.getFPGATime();
        String logEntry = String.format(
                "%f,%f,%f",
                timestamp,
                robotPose2d.getX(),
                robotPose2d.getY());
        return logEntry;

    }

    public Pose3d getAverageRobotPose3D() {
        return robotPoseAverager.getAveragePose();
    }

    public Pose2d getAverageRobotPose2D() {
        return robotPoseAverager.getAveragePose().toPose2d();
    }

    public Pose3d getAverageQuestPose3D() {
        return questPoseAverager.getAveragePose();
    }

    public Pose2d getAverageQuestPose2D() {
        return questPoseAverager.getAveragePose().toPose2d();
    }

    public void addVisionMeasurement(CommandSwerveDrivetrain drivetrain, Pose3d pose) {
        drivetrain.addVisionMeasurement(pose.toPose2d(), RobotController.getFPGATime(), Constants.Quest.QUESTNAV_STD_DEVS);
    }
    /**
     * Gets the Quest battery %
     * 
     * @return the Quest battery %
     */
    public Integer getQuestBattery() {
        return quest.getBatteryPercent().getAsInt();
    }

    /**
     * Class of quest commands
     */
    public class QuestCommands {
        public Command resetQuestPose(Pose3d newRobotPose) {
            return Commands.runOnce(() -> {
                Pose3d questPose = newRobotPose.transformBy(Constants.Quest.RobotToQuest3D);
                quest.setPose(questPose);
                questPoseAverager.reset();
                robotPoseAverager.reset();
            }).andThen(Commands.print("Reset Quest Pose!"));
        }

        public Command resetQuestPose(Pose2d newRobotPose) {
            return Commands.runOnce(() -> {
                Pose3d newRobotPose3d = new Pose3d(newRobotPose.getX(), newRobotPose.getY(), 0,
                        new Rotation3d(0, 0, newRobotPose.getRotation().getDegrees()));
                Pose3d questPose = newRobotPose3d.transformBy(Constants.Quest.RobotToQuest3D);

                quest.setPose(questPose);

                questPoseAverager.reset();
                robotPoseAverager.reset();
            });
        }

        // public Command updateNT() {
        // return new RunCommand(() -> {
        // networkTables.quest.updateQuestPose(questPose3d);
        // networkTables.quest.updateRobotPose(robotPose3d);
        // // networkTables.quest.updatePiPose(piPose3d);
        // });
        // }

    }
}
