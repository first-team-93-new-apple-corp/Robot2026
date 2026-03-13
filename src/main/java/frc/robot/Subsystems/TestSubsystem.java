package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Swerve;
import frc.robot.util.subsystems;

public class TestSubsystem extends SubsystemBase{
    private CommandSwerveDrivetrain drivetrain;
    public TestCommands commands = new TestCommands();

    public TestSubsystem(subsystems subsystems) {
        this.drivetrain = subsystems.drivetrain();
    }

    private void checkSwerveModules() {
        boolean allMotorsPresent = true;
        for (int module : Swerve.modules) {
            var currentModule = drivetrain.getModule(module);
            if (!(allMotorsPresent
                    && currentModule.getDriveMotor().isAlive()
                    && currentModule.getSteerMotor().isAlive()
                    && currentModule.getEncoder().isConnected())
                    && drivetrain.getPigeon2().isConnected()) {
                allMotorsPresent = false;
            }
        }
        SmartDashboard.putBoolean("Swerve Present?", allMotorsPresent);
    }

    @Override
    public void periodic() {
        checkSwerveModules();
    }

    public class TestCommands {
        public Command checkSwerve() {
                return Commands.run(() -> checkSwerveModules());
        }
    }

}
