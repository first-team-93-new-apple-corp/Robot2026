package frc.robot.util;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;

public class Logger extends DogLog {

  public static void log(TalonFX motor) {
    log(motor.getDescription() + "/StatorCurrent", motor.getStatorCurrent().getValue());
    log(motor.getDescription() + "/SupplyCurrent", motor.getSupplyCurrent().getValue());
    log(motor.getDescription() + "/Position", motor.getPosition().getValue());
    log(motor.getDescription() + "/Velocity", motor.getVelocity().getValue());
    log(motor.getDescription() + "/Temperature", motor.getDeviceTemp().getValue());
    log(motor.getDescription() + "/FaultField", motor.getFaultField().getStatus().toString());
    log(motor.getDescription() + "/StickyFaultField", motor.getStickyFaultField().getStatus().toString());
    log(motor.getDescription() + "/Connected", motor.isConnected());
    log(motor.getDescription() + "/Alive", motor.isAlive());
  }
    public static void log(CANcoder coder) {
    log(coder.getDeviceID() + "/Position", coder.getPosition().getValue());
    log(coder.getDeviceID() + "/Velocity", coder.getVelocity().getValue());
    log(coder.getDeviceID() + "/FaultField", coder.getFaultField().getStatus().toString());
    log(coder.getDeviceID() + "/StickyFaultField", coder.getStickyFaultField().getStatus().toString());
    log(coder.getDeviceID() + "/Connected", coder.isConnected());
    log(coder.getDeviceID() + "/MagnetHealth", coder.getMagnetHealth().getStatus());
  }
  public static void log(String key, Pose3d pose) {
    log(key + "/X", pose.getX(), "meters");
    log(key + "/Y", pose.getY(), "meters");
    log(key + "/Z", pose.getZ(), "meters");
    log(key + "/RotationX", pose.getRotation().getX(), "radians");
    log(key + "/RotationY", pose.getRotation().getY(), "radians");
    log(key + "/RotationZ", pose.getRotation().getZ(), "radians");
  }
}