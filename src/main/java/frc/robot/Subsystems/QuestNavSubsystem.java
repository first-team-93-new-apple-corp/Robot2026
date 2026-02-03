package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems.NTSubsystem.ntQuest;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    private QuestNav quest;

    // http://10.0.93.200:5801/

    public QuestCommands commands = new QuestCommands();
    private NTSubsystem networkTables;

    private Pose2d questPose2d;
    private Pose3d questPose3d;

    private Pose2d robotPose2d;
    private Pose3d robotPose3d;

    private Pose3d piPose3d;

    private PhotonCamera camera = new PhotonCamera("MainCam");
    private boolean hasPoseInit = false;
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(Inches.of(13), Inches.of(13.25), Inches.of(12)),
            new Rotation3d(Degrees.of(0), Degrees.of(22.5), Degrees.of(0)));
    private PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain, Pose3d startingPose, NTSubsystem networkTables) {
        this.drivetrain = drivetrain;
        quest = new QuestNav();

        // Assume this is the requested reset pose
        robotPose3d = startingPose;

        // Transform by the offset to get the Quest pose
        questPose3d = robotPose3d.transformBy(Constants.Quest.RobotToQuest);

        // Send the reset operation
        quest.setPose(questPose3d);

        this.networkTables = networkTables;

        quest.setVersionCheckEnabled(false);
    }

    public void setPose(Pose3d newRobotPose) {
        Pose3d questPose = newRobotPose.transformBy(Constants.Quest.RobotToQuest);
        quest.setPose(questPose);
    }

    @Override
    public void periodic() {
        quest.commandPeriodic();

        // if (!hasPoseInit) {
                Optional<EstimatedRobotPose> visionEst = Optional.empty();
                for (var result : camera.getAllUnreadResults()) {
                    visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
                    if (visionEst.isEmpty()) {
                        visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
                    }
                    visionEst.ifPresent(est -> {
                        piPose3d = est.estimatedPose;
                        // .transformBy(kRobotToCam.inverse());
                        
                        hasPoseInit = true;
                    });
                }
        // }

        SmartDashboard.putBoolean("Quest Connected", quest.isConnected());
        SmartDashboard.putBoolean("Quest Tracking?", quest.isTracking());
        SmartDashboard.putNumber("Quest Battery %", quest.getBatteryPercent().getAsInt());
        SmartDashboard.putBoolean("Has set?", hasPoseInit);

        if (quest.isTracking()) {
            // Get the latest pose data frames from the Quest
            PoseFrame[] questFrames = quest.getAllUnreadPoseFrames();

            // Loop over the pose data frames and send them to the pose estimator
            for (PoseFrame questFrame : questFrames) {
                // Get the pose of the Quest
                questPose3d = questFrame.questPose3d();
                questPose2d = questPose3d.toPose2d();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose
                robotPose3d = questPose3d.transformBy(Constants.Quest.RobotToQuest.inverse());
                robotPose2d = robotPose3d.toPose2d();

                // Add the measurement to our estimator
                drivetrain.addVisionMeasurement(robotPose2d, timestamp, Constants.Quest.QUESTNAV_STD_DEVS);

            }
        }

    }

    public class QuestCommands {
        public Command resetQuestPose(Pose3d newRobotPose) {
            return Commands.runOnce(() -> {
                Pose3d questPose = newRobotPose.transformBy(Constants.Quest.RobotToQuest);
                quest.setPose(questPose);

            }).andThen(Commands.print("Reset Quest Pose!"));
        }

        public Command resetQuestPose(Pose2d newRobotPose) {
            return Commands.runOnce(() -> {
                Pose3d newRobotPose3d = new Pose3d(newRobotPose.getX(), newRobotPose.getY(), 0,
                        new Rotation3d(0, 0, newRobotPose.getRotation().getDegrees()));
                Pose3d questPose = newRobotPose3d.transformBy(Constants.Quest.RobotToQuest);

                quest.setPose(questPose);
            });
        }

        public Command updateNT() {
            return new RunCommand(() -> {
                networkTables.quest.updateQuestPose(questPose3d);
                networkTables.quest.updateRobotPose(robotPose3d);
                networkTables.quest.updatePiPose(piPose3d);
            });
        }

    }
}
