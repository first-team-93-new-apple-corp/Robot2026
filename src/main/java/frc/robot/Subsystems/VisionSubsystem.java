package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import com.ctre.phoenix6.Utils;

// import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private boolean hasPiPoseData = false;
    private boolean resetting = true;
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, Constants.Photon.kRobotToCam);
    private VisionSystemSim visionSim;
    private SimCameraProperties cameraProp = new SimCameraProperties();
    private PhotonCameraSim cameraSim;
    // Stuff

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, NTSubsystem nt) {
        this.drivetrain = drivetrain;
        this.networkTables = nt;
        if (Utils.isSimulation()) {
            visionSim = new VisionSystemSim("Vision Simulation");
            visionSim.addAprilTags(kTagLayout);

            cameraProp.setCalibration(1200, 800, Rotation2d.fromDegrees(80));
            cameraProp.setCalibError(0.35, 0.08);
            cameraProp.setFPS(50);
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);

            cameraSim = new PhotonCameraSim(camera, cameraProp, kTagLayout);

            cameraSim.enableRawStream(true);
            cameraSim.enableProcessedStream(true);
            cameraSim.enableDrawWireframe(true);

            visionSim.addCamera(cameraSim, Constants.Photon.kRobotToCam);
        }

        // Quest Initialization
        quest = new QuestNav();
        quest.setVersionCheckEnabled(false);
        SmartDashboard.putData("Reset Vision", commands.resetPose().ignoringDisable(true));
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

        hasPoseInit = true;
        hasPiPoseData = false;

        // Reset our estimators with the new pose
        questPoseAverager.reset();
        robotPoseAverager.reset();
        piPoseAverager.reset();
    }

    public void smartDash() {
        SmartDashboard.putBoolean("Quest Connected", quest.isConnected());
        SmartDashboard.putBoolean("Quest Tracking?", quest.isTracking());
        SmartDashboard.putNumber("Quest Battery %", quest.getBatteryPercent().getAsInt());
        SmartDashboard.putNumber("Quest Tracking Lost", quest.getTrackingLostCounter().getAsInt());
        SmartDashboard.putBoolean("Has Pose Init?", hasPoseInit);
        SmartDashboard.putBoolean("Has Pi Data?", hasPiPoseData);
    }

    public void piPeriodic() {
        if (Utils.isSimulation()) {
            visionSim.update(drivetrain.getState().Pose);
        }
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
                        piPose3d = est.estimatedPose;
                        hasPiPoseData = true;
                    });
        }
    }

    /**
     * This is the method that should be called periodically to update quest
     * measurements
     */
    public void questPeriodic() {
        quest.commandPeriodic();

        if (hasPoseInit && !resetting) {
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
            if (piPose3d != null && hasPiPoseData && resetting) {
                quest.setPose(piPose3d.transformBy(Constants.Quest.RobotToQuest3D));
                hasPoseInit = true;
                resetting = false;
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
                setPose(newRobotPose);
                setPose(newRobotPose);
            }).andThen(Commands.print("Set Robot Pose!"));
        }

        public Command resetPose() {
            return Commands.runOnce(() -> {
                hasPiPoseData = false;
                hasPoseInit = false;
                resetting = true;
            });
        }

        public Command quest() {
            return Commands.runOnce(() -> {
                questPeriodic();
            });
        }

        public Command pi() {
            return Commands.runOnce(() -> {
                // if (hasPoseInit || resetting) {
                piPeriodic();
                // }
            });
        }

        public Command smartDashboard() {
            return Commands.run(() -> smartDash());
        }

    }
}
