package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Channels;

public class PowerDistributionSubsystem extends SubsystemBase {
    private final PowerDistribution powerDistribution;

    public PowerDistributionSubsystem() {
        powerDistribution = new PowerDistribution(Channels.POWER_DISTRIBUTION_CHANNEL,
                ModuleType.kRev);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Total Power via Hub",
                powerDistribution.getTotalPower());
        SmartDashboard.putNumber("Total Current via Hub",
                powerDistribution.getTotalCurrent());
        SmartDashboard.putNumber("Voltage via Hub", powerDistribution.getVoltage());
    }
}
