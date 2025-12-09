package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Swerve;

public class TestSubsystem {
    private CommandSwerveDrivetrain drivetrain;
    public TestCommands commands = new TestCommands();
    public TestSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public class TestCommands {
        public Command checkSwerve() {
            return Commands.runOnce(() -> {
                boolean allMotorsPresent = true;
                for (int module : Swerve.modules) {
                    var currentModule = drivetrain.getModule(module);
                    if (!(allMotorsPresent
                            && currentModule.getDriveMotor().isAlive()
                            && currentModule.getSteerMotor().isAlive()
                            && currentModule.getEncoder().isConnected())) {
                        allMotorsPresent = false;
                    }
                }
                SmartDashboard.putBoolean("Swerve Present?", allMotorsPresent);
            });
        }
    }

}
