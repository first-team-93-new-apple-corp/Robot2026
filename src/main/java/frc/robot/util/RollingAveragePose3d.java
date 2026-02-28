package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.LinkedList;
import java.util.Queue;

public class RollingAveragePose3d {
  private final int windowSize;
  private final Queue<Pose3d> poses;

  private double sumX;
  private double sumY;
  private double sumZ;
  // Sum quaternion components to average orientation in 3D
  private double sumQw;
  private double sumQx;
  private double sumQy;
  private double sumQz;

  public RollingAveragePose3d(int windowSize) {
    this.windowSize = windowSize;
    this.poses = new LinkedList<>();

    sumX = 0.0;
    sumY = 0.0;
    sumZ = 0.0;
    sumQw = 0.0;
    sumQx = 0.0;
    sumQy = 0.0;
    sumQz = 0.0;
  }

  public void addPose(Pose3d pose) {
    poses.add(pose);

    sumX += pose.getX();
    sumY += pose.getY();
    sumZ += pose.getZ();

    Quaternion q = pose.getRotation().getQuaternion();
    sumQw += q.getW();
    sumQx += q.getX();
    sumQy += q.getY();
    sumQz += q.getZ();

    if (poses.size() > windowSize) {
      Pose3d removed = poses.poll();
      sumX -= removed.getX();
      sumY -= removed.getY();
      sumZ -= removed.getZ();

      Quaternion rq = removed.getRotation().getQuaternion();
      sumQw -= rq.getW();
      sumQx -= rq.getX();
      sumQy -= rq.getY();
      sumQz -= rq.getZ();
    }
  }

  public Pose3d getAveragePose() {
    if (poses.isEmpty()) {
      return new Pose3d(); // default zero pose
    }

    int size = poses.size();
    double avgX = sumX / size;
    double avgY = sumY / size;
    double avgZ = sumZ / size;

    // Average quaternion components then normalize to unit quaternion
    double aw = sumQw / size;
    double ax = sumQx / size;
    double ay = sumQy / size;
    double az = sumQz / size;

    double norm = Math.sqrt(aw * aw + ax * ax + ay * ay + az * az);
    if (norm > 1e-12) {
      aw /= norm;
      ax /= norm;
      ay /= norm;
      az /= norm;
    } else {
      // fallback to identity rotation
      aw = 1.0;
      ax = ay = az = 0.0;
    }

    Quaternion avgQuat = new Quaternion(ax, ay, az, aw); // (x,y,z,w)
    Rotation3d avgRot = new Rotation3d(avgQuat);

    return new Pose3d(avgX, avgY, avgZ, avgRot);
  }

  public void reset() {
    poses.clear();
    sumX = 0.0;
    sumY = 0.0;
    sumZ = 0.0;
    sumQw = 0.0;
    sumQx = 0.0;
    sumQy = 0.0;
    sumQz = 0.0;
  }
}