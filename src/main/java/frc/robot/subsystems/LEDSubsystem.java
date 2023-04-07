package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
        private final AddressableLED m_led = new AddressableLED(6);
        private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(20);

        public LEDSubsystem() {
                m_led.setLength(m_ledBuffer.getLength());
                m_led.setData(m_ledBuffer);
                m_led.start();

                for (int i=0;i<m_ledBuffer.getLength();i++) {
                        m_ledBuffer.setRGB(i, 255,0,255);
                }
                m_led.setData(m_ledBuffer);
        }
}
