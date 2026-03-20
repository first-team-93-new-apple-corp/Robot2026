package frc.robot.Subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

// import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    private boolean hasPiPoseData = true;
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, Constants.Photon.kRobotToCam);

    // Stuff

    private Matrix<N3, N1> curStdDevs;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, NTSubsystem nt) {
        this.drivetrain = drivetrain;
        this.networkTables = nt;
        // Quest Initialization
        quest = new QuestNav();
        quest.setVersionCheckEnabled(false);
    }

    /**
     * Sets a new pose and sets all our vision devices to that pose (with
     * transforms)
     * 
     * @param newRobotPose new pose to set
     */
    public void setPose(Pose3d newRobotPose) {
        Pose3d questPose = newRobotPose.transformBy(Constants.Quest.RobotToQuest3D);
        // Pose3d piPose = newRobotPose.transformBy(Constants.Photon.kRobotToCam);
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
        SmartDashboard.putBoolean("Has Pose Init?", hasPoseInit);
        SmartDashboard.putBoolean("Has Pi Data?", hasPiPoseData);

        // PhotonVision Estimation
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
                hasPiPoseData = false;
            }
            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        // var estStdDevs = getEstimationStdDevs();
                        // drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, Constants.Photon.standardDevs);
                        piPose3d = est.estimatedPose;
                        hasPiPoseData = true;
                    });
        }
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
                // Clears the queue of pose frames to prevent old data from being processed when
                // the Quest starts up
                quest.getAllUnreadPoseFrames();
            }
            if (piPose3d != null && hasPiPoseData) {
                quest.setPose(piPose3d.transformBy(Constants.Quest.RobotToQuest3D));
                hasPoseInit = true;
            }
        }

        // Update NetworkTables
        networkTables.quest.updateQuestPose(questPose3d);
        networkTables.quest.updateRobotPose(drivetrain.getState().Pose);
        networkTables.quest.updatePiPose(piPose3d);

        // networkTables.quest.updateAvgRobotPose(getAverageRobotPose3D());

        // Pose Averaging
        questPoseAverager.addPose(questPose3d);
        robotPoseAverager.addPose(robotPose3d);
        piPoseAverager.addPose(piPose3d);
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

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.Photon.singleTagDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.Photon.singleTagDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = Constants.Photon.singleTagDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = Constants.Photon.multiTagDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public Pose3d getAverageRobotPose3D() {
        return robotPoseAverager.getAveragePose();
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
