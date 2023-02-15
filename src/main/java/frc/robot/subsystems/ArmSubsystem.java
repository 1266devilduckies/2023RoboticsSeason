package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
        
        private double m_armKp = 0;
        private double m_armKg = 0;
        public WPI_TalonFX armMotor = new WPI_TalonFX(Constants.CAN.Arm.arm); 
        public TalonFXSimCollection armMotorSim;
        SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 
                Constants.Arm.gearing, SingleJointedArmSim.estimateMOI(Constants.Arm.armLengthMeters, Constants.Arm.armMassKg), 
                Constants.Arm.armLengthMeters, -6.28, 6.28, true);

        private final PIDController m_controller = new PIDController(m_armKp, 0, 0);
        private double m_armSetpointDegrees = 0;

        private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
        
        private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
        private final MechanismLigament2d m_armTower =
                m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
        private final MechanismLigament2d m_arm =
                m_armPivot.append(
                new MechanismLigament2d(
                        "Arm",
                        30,
                        Units.radiansToDegrees(armSim.getAngleRads()),
                        6,
                        new Color8Bit(Color.kYellow)));

        public ArmSubsystem(){
                SmartDashboard.putData("Arm Sim", m_mech2d);
                m_armTower.setColor(new Color8Bit(Color.kBlue));
                armMotorSim = armMotor.getSimCollection();

                Preferences.initDouble(Constants.Arm.kArmPositionKey, m_armSetpointDegrees);
                Preferences.initDouble(Constants.Arm.kArmPKey, m_armKp);
                Preferences.initDouble(Constants.Arm.kArmGKey, m_armKg);
        
        }

        @Override
        public void periodic() {
                armMotor.set(ControlMode.PercentOutput, m_armKg);
        }

        @Override
        public void simulationPeriodic() {
                // In this method, we update our simulation of what our arm is doing
                // First, we set our "inputs" (voltages)
                armSim.setInput(armMotor.get() * RobotController.getBatteryVoltage());
            
                // Next, we update it. The standard loop time is 20ms.
                armSim.update(0.020);
            
                // Finally, we set our simulated encoder's readings and simulated battery voltage
                armMotorSim.setIntegratedSensorRawPosition((int) ((armSim.getAngleRads() / 6.28) * Constants.Arm.gearing * 2048.0));
                // SimBattery estimates loaded battery voltages
                RoboRioSim.setVInVoltage(
                    BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
            
                // Update the Mechanism Arm angle based on the simulated arm angle
                m_arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
        }     

        public void loadPreferences() {
                // Read Preferences for Arm setpoint and kP on entering Teleop
                m_armSetpointDegrees = Preferences.getDouble(Constants.Arm.kArmPositionKey, m_armSetpointDegrees);
                if (m_armKp != Preferences.getDouble(Constants.Arm.kArmPKey, m_armKp)) {
                  m_armKp = Preferences.getDouble(Constants.Arm.kArmPKey, m_armKp);
                  m_controller.setP(m_armKp);
                }
                if(m_armKg != Preferences.getDouble(Constants.Arm.kArmGKey, m_armKg)){
                        m_armKg = Preferences.getDouble(Constants.Arm.kArmGKey, m_armKg);    
                }
        }
}
