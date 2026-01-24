package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NTSubsystem extends SubsystemBase {
    private final Field2d m_field = new Field2d();

    public NTSubsystem(Pose2d robotPose, Pose2d questPose) {
        SmartDashboard.putData("Field", m_field);
        m_field.setRobotPose(robotPose);
        m_field.getObject("Quest").setPose(questPose);
    }

    public NTSubsystem(Pose3d robotPose, Pose3d questPose) {
        SmartDashboard.putData("Field", m_field);
        m_field.setRobotPose(robotPose.toPose2d());
        m_field.getObject("Quest").setPose(questPose.toPose2d());
    }

    public void updateRobotPose(Pose2d newPose) {
        m_field.setRobotPose(newPose);
    }

    public void updateRobotPose(Pose3d newPose) {
        m_field.setRobotPose(newPose.toPose2d());
    }

    public void updateQuestPose(Pose2d newPose) {
        m_field.getObject("Quest").setPose(newPose);
    }

    public void updateQuestPose(Pose3d newPose) {
        m_field.getObject("Quest").setPose(newPose.toPose2d());

    }
}
