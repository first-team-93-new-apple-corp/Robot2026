package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RollingAverageDouble;

public class PowerDistributionSubsystem extends SubsystemBase{

    public final PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);
    public final RollingAverageDouble currentAverage = new RollingAverageDouble(25);

    public double totalCurrent;
    public double voltage;

    @Override
    public void periodic() {
        currentAverage.addValue(PDH.getTotalCurrent());
        // PDH.
        voltage = PDH.getVoltage();
        SmartDashboard.putNumber("Total Current", currentAverage.getAverage());
        SmartDashboard.putNumber("Voltage", voltage);
    }
}