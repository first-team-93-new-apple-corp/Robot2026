package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerDistributionSubsystem extends SubsystemBase{

    public final PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);

    public double totalCurrent;
    public double voltage;

    @Override
    public void periodic() {
        totalCurrent = PDH.getTotalCurrent();
        voltage = PDH.getVoltage();
        SmartDashboard.putNumber("Total Current", totalCurrent);
        SmartDashboard.putNumber("Voltage", voltage);
    }
}