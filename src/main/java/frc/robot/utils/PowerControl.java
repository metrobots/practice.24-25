package frc.robot.utils;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;


public class PowerControl {

    PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);
    
    double voltage = PDH.getVoltage();
    double temperatureCelsius = PDH.getTemperature();

    


    public double getChannelVoltage(int channel) {
        double current = PDH.getCurrent(channel);
        return current;
    }
    public void channelOn() {
        PDH.setSwitchableChannel(true);
    }
    public void channelOff() {
        PDH.setSwitchableChannel(false);
    }
}
