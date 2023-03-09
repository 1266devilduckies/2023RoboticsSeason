package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DuckAHRS;
import frc.robot.DuckGearUtil;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommand;

public class DrivetrainSubsystem extends SubsystemBase {
  public final DifferentialDrive robotDrive;

  // CAN devices
  public final WPI_TalonFX MainLeftMotorBack = new WPI_TalonFX(Constants.CAN.Drivetrain.BL);
  public final  WPI_TalonFX MainRightMotorBack = new WPI_TalonFX(Constants.CAN.Drivetrain.BR);
  public final WPI_TalonFX MainLeftMotorFront = new WPI_TalonFX(Constants.CAN.Drivetrain.FL);
  public final WPI_TalonFX MainRightMotorFront = new WPI_TalonFX(Constants.CAN.Drivetrain.FR);
  public final WPI_TalonFX leftTopMotor = new WPI_TalonFX(Constants.CAN.Drivetrain.TL);
  public final WPI_TalonFX rightTopMotor = new WPI_TalonFX(Constants.CAN.Drivetrain.TR);
  
  public DuckAHRS gyro = new DuckAHRS();
  public final RamseteController ramseteController = new RamseteController();
  public final DifferentialDriveKinematics drivetrainKinematics = new DifferentialDriveKinematics(Constants.DrivetrainCharacteristics.trackWidthMeters);

  private final DifferentialDrivePoseEstimator odometry;

  public static final Field2d field = new Field2d();
  public boolean isCurrentLimited = false;
  private double gyroPitchkP = 0.033;

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DrivetrainCharacteristics.kS, Constants.DrivetrainCharacteristics.kV, Constants.DrivetrainCharacteristics.kA);
  
  //proportional controllers only gang uwu
  public PIDController pidGyroPitch = new PIDController(gyroPitchkP, 0.0, 0.0);

  public DrivetrainSubsystem(RobotContainer robotContainer) {
    //constructor gets ran at robotInit()
    this.setDefaultCommand(new DriveCommand(this));

    // Reset settings
    MainLeftMotorBack.configFactoryDefault();
    MainRightMotorBack.configFactoryDefault();
    MainLeftMotorFront.configFactoryDefault();
    MainRightMotorFront.configFactoryDefault();
    leftTopMotor.configFactoryDefault();
    rightTopMotor.configFactoryDefault();

    // Setup the integrated sensor
    MainLeftMotorBack.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    MainRightMotorBack.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);

    // Slave the front motors to their respective back motors
    MainLeftMotorFront.follow(MainLeftMotorBack);
    MainRightMotorFront.follow(MainRightMotorBack);
    leftTopMotor.follow(MainLeftMotorBack);
    rightTopMotor.follow(MainRightMotorBack);
    
    MainLeftMotorBack.enableVoltageCompensation(false);
    MainLeftMotorFront.enableVoltageCompensation(false);
    MainRightMotorBack.enableVoltageCompensation(false);
    MainRightMotorFront.enableVoltageCompensation(false);
    leftTopMotor.enableVoltageCompensation(false);
    rightTopMotor.enableVoltageCompensation(false);

    //Apply brake mode in auton
    MainLeftMotorBack.setNeutralMode(NeutralMode.Brake);
    MainLeftMotorFront.setNeutralMode(NeutralMode.Brake);
    MainRightMotorBack.setNeutralMode(NeutralMode.Brake);
    MainRightMotorFront.setNeutralMode(NeutralMode.Brake);
    leftTopMotor.setNeutralMode(NeutralMode.Brake);
    rightTopMotor.setNeutralMode(NeutralMode.Brake);

    // Invert one of the sides
    MainLeftMotorBack.setInverted(true);
    MainRightMotorBack.setInverted(false);
    MainLeftMotorFront.setInverted(InvertType.FollowMaster);
    MainRightMotorFront.setInverted(InvertType.FollowMaster);
    leftTopMotor.setInverted(InvertType.FollowMaster);
    rightTopMotor.setInverted(InvertType.FollowMaster);

    //configure PIDF values
    MainLeftMotorBack.config_kP(0, 0.0);
    MainRightMotorBack.config_kP(0, 0.0);

    robotDrive = new DifferentialDrive(MainLeftMotorBack, MainRightMotorBack);
    
    odometry = new DifferentialDrivePoseEstimator(
        drivetrainKinematics, 
        Rotation2d.fromDegrees(-gyro.getAngle()), 
        0,0,
        new Pose2d());

    robotDrive.setDeadband(Constants.DrivetrainCharacteristics.deadband);
    setCurrentLimit(true);
  }

  @Override
  public void periodic() {
    double leftDistanceMeters = DuckGearUtil.encoderTicksToMeters(MainLeftMotorBack.getSelectedSensorPosition(),
    Constants.DrivetrainCharacteristics.gearing, 2048.0, Constants.DrivetrainCharacteristics.wheelRadiusMeters);
    double rightDistanceMeters = DuckGearUtil.encoderTicksToMeters(MainRightMotorBack.getSelectedSensorPosition(),
    Constants.DrivetrainCharacteristics.gearing, 2048.0, Constants.DrivetrainCharacteristics.wheelRadiusMeters);

    odometry.update(Rotation2d.fromDegrees(-gyro.getAngle()), leftDistanceMeters, rightDistanceMeters);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        DuckGearUtil.EncoderTicksPer100msToMetersPerSecond(MainLeftMotorBack.getSelectedSensorVelocity(),
            Constants.DrivetrainCharacteristics.gearing, 2048.0, Constants.DrivetrainCharacteristics.wheelRadiusMeters),
        DuckGearUtil.EncoderTicksPer100msToMetersPerSecond(MainRightMotorBack.getSelectedSensorVelocity(),
            Constants.DrivetrainCharacteristics.gearing, 2048.0, Constants.DrivetrainCharacteristics.wheelRadiusMeters));
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void addApriltagMeasurement(Pose2d guessPoseMeters, double timeStamp) {
        odometry.addVisionMeasurement(guessPoseMeters, timeStamp);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    //flipping voltage for some reason, it works better???
    MainLeftMotorBack.setVoltage(leftVolts);
    MainRightMotorBack.setVoltage(rightVolts);
    robotDrive.feed(); // feed watchdog to prevent error from clogging can bus
  }

  public void encoderBasedDrive(double leftMotorPercentage, double rightMotorPercentage){

    double leftTickSpeed = Constants.DrivetrainCharacteristics.maxSpeedTicks * leftMotorPercentage;
    double rightTickSpeed = Constants.DrivetrainCharacteristics.maxSpeedTicks * rightMotorPercentage;

    double leftSpeedMeters = DuckGearUtil.EncoderTicksPer100msToMetersPerSecond(leftTickSpeed, 
        Constants.DrivetrainCharacteristics.gearing, 2048.0, Constants.DrivetrainCharacteristics.wheelRadiusMeters);

    double rightSpeedMeters = DuckGearUtil.EncoderTicksPer100msToMetersPerSecond(rightTickSpeed, 
        Constants.DrivetrainCharacteristics.gearing, 2048.0, Constants.DrivetrainCharacteristics.wheelRadiusMeters);

    MainLeftMotorBack.set(ControlMode.Velocity, leftTickSpeed, DemandType.ArbitraryFeedForward, feedforward.calculate(leftSpeedMeters) / RobotController.getBatteryVoltage());
    MainRightMotorBack.set(ControlMode.Velocity, rightTickSpeed, DemandType.ArbitraryFeedForward, feedforward.calculate(rightSpeedMeters) / RobotController.getBatteryVoltage());
    robotDrive.feed();
  }

  public void resetOdometry(Pose2d pose) {
    gyro.reset();
    resetEncoders();

    odometry.resetPosition(Rotation2d.fromDegrees(-gyro.getAngle()),0.,0., pose);
  }

  public void resetEncoders() {
    MainLeftMotorBack.setSelectedSensorPosition(0);
    MainRightMotorBack.setSelectedSensorPosition(0);
  }

  public void setCurrentLimit(boolean isCurrentLimited) {
    int currentLimitAmp = 30;
    SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(isCurrentLimited, currentLimitAmp, 60, 1.0);
    
    MainLeftMotorBack.configSupplyCurrentLimit(supplyLimit, 100);
    MainRightMotorBack.configSupplyCurrentLimit(supplyLimit, 100);
  }
}
