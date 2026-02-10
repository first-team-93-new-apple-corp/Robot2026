package frc.robot.Subsystems;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.RollingAveragePose2d;
import frc.robot.util.RollingAveragePose3d;

/**
 * Interface with the QuestNav on VR headset for pose estimation. See
 * https://www.chiefdelphi.com/t/questnav-the-best-robot-pose-tracking-system-in-frc/
 * Stolen from 1732 mwehehehehe >:)
 */

public class QuestNav {
  // Configure Network Tables topics (questnav/...) to communicate with the Quest
  // HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position")
      .subscribe(new float[] { 0.0f, 0.0f, 0.0f });
  private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion")
      .subscribe(new float[] { 0.0f, 0.0f, 0.0f, 0.0f });
  private FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles")
      .subscribe(new float[] { 0.0f, 0.0f, 0.0f });
  private DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  // Pose of the Quest when the pose was reset
  private Pose2d resetPoseQuest2D = new Pose2d();
  private Pose3d resetPoseQuest3D = new Pose3d();

  // Pose of the robot when the pose was reset
  private Pose2d resetPoseRobot2D = new Pose2d();
  private Pose3d resetPoseRobot3D = new Pose3d();

  private final Transform2d robotToQuest2D = Constants.Quest.RobotToQuest2D;
  private final Transform3d robotToQuest3D = Constants.Quest.RobotToQuest3D;

  private final RollingAveragePose2d rollingAvg2d;
  private final RollingAveragePose3d rollingAvg3d;

  /*** Constructor */
  public QuestNav(int windowSize) {
    // Zero the absolute 3D position of the robot (similar to long-pressing the
    // quest logo)
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
    rollingAvg2d = new RollingAveragePose2d(windowSize);
    rollingAvg3d = new RollingAveragePose3d(windowSize);
  }

  public QuestNav() {
    this(2);
  }

  public void updateAverageRobotPose2D() {
    rollingAvg2d.addPose(getRobotPose2D());
  }

  public void updateAverageRobotPose3D() {
    rollingAvg3d.addPose(getRobotPose3D());
  }

  public Pose2d getAverageRobotPose2D() {
    return rollingAvg2d.getAveragePose();
  }

  public Pose3d getAverageRobotPose3D() {
    return rollingAvg3d.getAveragePose();
  }

  /**
   * Gets the pose of the robot on the field
   *
   * @return pose of the robot
   */
  public Pose2d getRobotPose2D() {
    // The robot is the Quest's pose transformed back by the quest->robot offset
    return getQuestPose2D().transformBy(robotToQuest2D.inverse());
  }

  public Pose3d getRobotPose3D() {
    // The robot is the Quest's pose transformed back by the quest->robot offset
    return getQuestPose3D().transformBy(robotToQuest3D.inverse());
  }

  /**
   * Gets the pose of the Quest on the field
   *
   * @return pose of the Quest
   */
  public Pose2d getQuestPose2D() {
    var rawPose = getUnfilteredQuestPose2D();
    var poseRelativeToReset = rawPose.minus(resetPoseQuest2D);
    // Transform from "reset quest pose" to "current quest pose"

    return resetPoseRobot2D // the robot's field pose at reset
        .transformBy(robotToQuest2D) // offset to get the Quest's field pose at reset
        .transformBy(poseRelativeToReset);
  }

  public Pose3d getQuestPose3D() {
    var rawPose = getUnfilteredQuestPose3D();
    var poseRelativeToReset = rawPose.minus(resetPoseQuest3D);
    // Transform from "reset quest pose" to "current quest pose"

    return resetPoseRobot3D // the robot's field pose at reset
        .transformBy(robotToQuest3D) // offset to get the Quest's field pose at reset
        .transformBy(poseRelativeToReset);
  }

  /*
   * Gets the battery percent of the Quest.
   *
   * @return battery percent of the Quest
   */
  public double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  /**
   * Returns if the Quest is connected
   *
   * @return true if the Quest is connected
   */
  public boolean isConnected() {
    // System.out.println("FPGATime: " + RobotController.getFPGATime());
    // System.out.println("Last Change: " + questBatteryPercent.getLastChange());
    // System.out.println(
    // "Diff: " + (RobotController.getFPGATime() -
    // questBatteryPercent.getLastChange()) /
    // 1000.0);
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000.0) < 30.0;
  }

  /**
   * Gets the raw Rotation3d of the Quest
   *
   * @return Rotation3d of the Quest, not adjusted for the reset pose
   */
  public Rotation3d getQuaternion() {
    float[] qqFloats = questQuaternion.get();
    return new Rotation3d(new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]));
  }

  /**
   * Gets the Quests's timestamp
   *
   * @return the Quest timestamp
   */
  public double getTimestamp() {
    return questTimestamp.get();
  }

  /**
   * Set the robot's pose on the field. This is useful to seed the robot to a
   * known position. This
   * is usually called at the start of the autonomous period.
   *
   * @param newPose new robot pose
   */
  public void resetPose2D(Pose2d newPose) {
    // rollingAvg2d.reset();
    resetPoseQuest2D = getQuestPose2D();
    resetPoseRobot2D = newPose;
  }

  public void resetPose3D(Pose3d newPose) {
    // rollingAvg2d.reset();
    resetPoseQuest3D = getQuestPose3D();
    resetPoseRobot3D = newPose;
  }

  /**
   * Clean up questnav subroutine messages after processing on the headset. Call
   * this each iteration
   * to reset the command after it has been processed.
   */
  public void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  /**
   * Gets the raw pose of the Quest, relative to the position where it started
   *
   * @return pose of the Quest
   */
  private Pose2d getUnfilteredQuestPose2D() {
    var eulerAngles = questEulerAngles.get();
    var rotation = Rotation2d.fromDegrees(-Math.IEEEremainder(eulerAngles[1], 360d));

    var questnavPosition = questPosition.get();
    var translation = new Translation2d(questnavPosition[2], -questnavPosition[0]);
    return new Pose2d(translation, rotation);
  }


  // TODO: Test this, generated by github copilot, may be wrong. my half baked attempt is commented out below
  /**
   * Gets the raw pose of the Quest, relative to the position where it started
   *
   * @return pose of the Quest
   */
  private Pose3d getUnfilteredQuestPose3D() {
    // Use the raw quaternion for full 3D orientation
    var quaternion = getQuaternion();

    var questnavPosition = questPosition.get();
    var translation = new Translation3d(questnavPosition[2], -questnavPosition[0], questnavPosition[1]);

    return new Pose3d(translation, quaternion);
  }
    // private Pose3d getUnfilteredQuestPose3D() {
  //   // var eulerAngles = questEulerAngles.get();
  //   var quaternion = questQuaternion.get();

  //   var rotation = Rotation3d.fromDegrees(-Math.IEEEremainder(eulerAngles[1], 360d));

  //   var questnavPosition = questPosition.get();
  //   var translation = new Translation3d(questnavPosition[2], -questnavPosition[0], questnavPosition[1]);
  //   return new Pose3d(translation, rotation);
  // }
}