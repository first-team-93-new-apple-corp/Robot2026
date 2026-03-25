package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;

public class Logger extends DogLog {

  public static void log(TalonFX motor) {
    log(motor.getDescription() + "/StatorCurrent", motor.getStatorCurrent().getValue());
    log(motor.getDescription() + "/Position", motor.getPosition().getValue());
    log(motor.getDescription() + "/Velocity", motor.getVelocity().getValue());
  }
}