package frc.robot.subsystems;

import java.awt.image.BufferedImage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {

        private DigitalInput inputData0;

        public ColorSensor(){
                inputData0 = new DigitalInput(0);
        }

        @Override
        public void periodic() {
                
        }

        public boolean getSensed(){
                return inputData0.get();
        }        
}
