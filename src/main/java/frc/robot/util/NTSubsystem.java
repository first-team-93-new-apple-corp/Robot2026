package frc.robot.util;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class NTSubsystem {

	// Our NT
	private NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
	// private NetworkTable questInst = ntInst.getTable("questnav");

	// Classes
	public ntSwerve swerve = new ntSwerve();
	public ntQuest quest = new ntQuest();

	// Published things
	private final Field2d m_field = new Field2d();

	// Constructors
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

	public class ntSwerve {
		// From CTRE Telemtry.java file (might be modified)

		// Gets NT (Change to change what table is called)
		private final NetworkTable driveStateTable = ntInst.getTable("DriveState");

		// Drive pose, speed, module states, module targets, module positions,
		// timestamp, odom publishers
		private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct)
				.publish();
		private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable
				.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
		private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable
				.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
		private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable
				.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
		private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable
				.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
		private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
		private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency")
				.publish();

		/* Robot pose for field positioning */
		private final NetworkTable table = ntInst.getTable("Pose");
		private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
		private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

		/* Mechanisms to represent the swerve module states */
		private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] { new Mechanism2d(1, 1),
				new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1), };

		/* A direction and length changing ligament for speed representation */
		private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
				m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
				m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
				m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
				m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5)
						.append(new MechanismLigament2d("Speed", 0.5, 0)), };

		/* A direction changing and length constant ligament for module direction */
		private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
				m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
						.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
				m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
						.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
				m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
						.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
				m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
						.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))), };

		public ntSwerve() {
			/* Set up the module state Mechanism2d telemetry */
			for (int i = 0; i < 4; ++i) {
				SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
			}

		}

		private final double[] m_poseArray = new double[3];

		/**
		 * Accept the swerve drive state and telemeterize it to SmartDashboard and
		 * SignalLogger.
		 */
		public void telemeterize(SwerveDriveState state) {
			/* Telemeterize the swerve drive state */
			drivePose.set(state.Pose);
			driveSpeeds.set(state.Speeds);
			driveModuleStates.set(state.ModuleStates);
			driveModuleTargets.set(state.ModuleTargets);
			driveModulePositions.set(state.ModulePositions);
			driveTimestamp.set(state.Timestamp);
			driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

			/* Also write to log file */
			SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
			SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
			SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
			SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
			SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct,
					state.ModulePositions);
			SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

			/* Telemeterize the pose to a Field2d */
			fieldTypePub.set("Field2d");

			m_poseArray[0] = state.Pose.getX();
			m_poseArray[1] = state.Pose.getY();
			m_poseArray[2] = state.Pose.getRotation().getDegrees();
			fieldPub.set(m_poseArray);

			/* Telemeterize each module state to a Mechanism2d */
			for (int i = 0; i < 4; ++i) {
				m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
				m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
				m_moduleSpeeds[i]
						.setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * Constants.Swerve.MaxSpeed));
			}
		}
	}

	public class ntQuest {

		public void updateQuestPose(Pose2d pose) {
			// return Commands.runOnce(() -> {
			m_field.getObject("Quest").setPose(pose);
			// System.out.println("Reset Robot NT Pose");
			// });
		}

		public void updateQuestPose(Pose3d pose) {
			m_field.getObject("Quest").setPose(pose.toPose2d());
			// System.out.println("Reset Robot NT Pose");
		}

		public void updateAvgQuestPose(Pose2d pose) {
			// return Commands.runOnce(() -> {
			m_field.getObject("QuestAvg").setPose(pose);
			// System.out.println("Reset Robot NT Pose");
			// });
		}

		public void updateAvgQuestPose(Pose3d pose) {
			m_field.getObject("QuestAvg").setPose(pose.toPose2d());
			// System.out.println("Reset Robot NT Pose");
		}

		public void updateAvgRobotPose(Pose2d pose) {
			// return Commands.runOnce(() -> {
			m_field.getObject("RobotAvg").setPose(pose);
			// System.out.println("Reset Robot NT Pose");
			// });
		}

		public void updateAvgRobotPose(Pose3d pose) {
			m_field.getObject("RobotAvg").setPose(pose.toPose2d());
			// System.out.println("Reset Robot NT Pose");
		}

		public void updateRobotPose(Pose2d pose) {
			// return new InstantCommand(() -> {
			m_field.setRobotPose(pose);
			// System.out.println("Reset Robot NT Pose");
			// });

		}

		public void updateRobotPose(Pose3d pose) {
			// return new InstantCommand(() -> {
			m_field.setRobotPose(pose.toPose2d());
			// System.out.println("Reset Robot NT Pose");
			// });
		}

		public void updatePiPose(Pose3d pose) {
			m_field.getObject("PI").setPose(pose.toPose2d());
		}
	}
}
