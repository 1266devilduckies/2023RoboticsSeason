package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;


public class DuckAHRS {
        private AHRS actualGyro;
        public DuckAHRS() {
                try {
                        /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
                        /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
                        /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
                        actualGyro = new AHRS(SerialPort.Port.kMXP);
                        System.out.println("fjdshafklssadfsad");
                    } catch (RuntimeException ex ) {
                        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
                    }
        }

        public double getAngle() {
                //yaw (relative to robot)
                if (actualGyro != null) {
                        return (double)actualGyro.getPitch();
                } else {
                        return 0.0;
                }
        }
        public void reset() {
                if (actualGyro != null) {
                        actualGyro.reset();
                }
        }
        public double getPitch() {
                if (actualGyro != null) {
                        return (double)actualGyro.getYaw();
                } else {
                        return 0.0;
                }
        }
}
