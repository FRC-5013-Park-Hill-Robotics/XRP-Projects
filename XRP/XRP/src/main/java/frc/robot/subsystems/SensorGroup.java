package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;

public class SensorGroup {
    public final AnalogInput m_rangeFinder = new AnalogInput(2);
    public final AnalogInput m_lineFollower = new AnalogInput(1);
}
