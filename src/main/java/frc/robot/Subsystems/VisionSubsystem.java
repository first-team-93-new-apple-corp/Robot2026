package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

// import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.NTSubsystem;
import frc.robot.util.RollingAveragePose3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class VisionSubsystem extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    private QuestNav quest;
    // http://10.0.93.200:5801/

    // Quest Commands
    public QuestCommands commands = new QuestCommands();

    // Network Tables
    private NTSubsystem networkTables;

    // Last poses
    private Pose3d questPose3d = new Pose3d();
    private Pose3d robotPose3d = new Pose3d();
    private Pose3d piPose3d = new Pose3d();

    // Pose Averagers
    private RollingAveragePose3d robotPoseAverager = new RollingAveragePose3d(10);
    private RollingAveragePose3d questPoseAverager = new RollingAveragePose3d(10);
    private RollingAveragePose3d piPoseAverager = new RollingAveragePose3d(10);

    // PhotonVision
    private PhotonCamera camera = new PhotonCamera("MainCam");
    private boolean hasPoseInit = false;
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, Constants.Photon.kRobotToCam);

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, NTSubsystem nt) {
        this.drivetrain = drivetrain;
        this.networkTables = nt;
        // Quest Initialization
        quest = new QuestNav();
        quest.setVersionCheckEnabled(false);
    }

    /**
     * Sets a new pose and sets all our vision devices to that pose (with transforms)
     * @param newRobotPose new pose to set
     */
    public void setPose(Pose3d newRobotPose) {
        Pose3d questPose = newRobotPose.transformBy(Constants.Quest.RobotToQuest3D);
        Pose3d piPose = newRobotPose.transformBy(Constants.Photon.kRobotToCam);
        quest.setPose(questPose);

        // Reset our estimators with the new pose
        questPoseAverager.reset();
        robotPoseAverager.reset();
        piPoseAverager.reset();
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
        
        if (hasPoseInit) { // Skip quest if it hasn't started up yet
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

                    // Add the measurement to our estimator
                    addVisionMeasurement(drivetrain, robotPose3d);

                }
            }
        } else {
            if (quest.isTracking()) {
                // Clears the queue of pose frames to prevent old data from being processed when the Quest starts up
                PoseFrame[] questFrames = quest.getAllUnreadPoseFrames();
            }
        }

        // Update NetworkTables
        networkTables.quest.updateQuestPose(questPose3d);
        networkTables.quest.updateRobotPose(robotPose3d);
        networkTables.quest.updateAvgQuestPose(getAverageQuestPose3D());
        networkTables.quest.updateAvgRobotPose(getAverageRobotPose3D());

        // Pose Averaging
        questPoseAverager.addPose(questPose3d);
        robotPoseAverager.addPose(robotPose3d);

        // PhotonVision Estimation
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }

            visionEst.ifPresent(
                    est -> {
                        piPose3d = est.estimatedPose;
                    });
        }
    }

    public String questPoseInfo() {
        double timestamp = RobotController.getFPGATime();
        String logEntry = String.format(
                "%f,%f,%f",
                timestamp,
                robotPose3d.getX(),
                robotPose3d.getY());
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
        drivetrain.addVisionMeasurement(pose.toPose2d(), RobotController.getFPGATime(),
                Constants.Quest.QUESTNAV_STD_DEVS);
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
        public Command setRobotPose(Pose3d newRobotPose) {
            return Commands.runOnce(() -> {
                setRobotPose(newRobotPose);
            }).andThen(Commands.print("Set Robot Pose!"));
        }
    }
}
