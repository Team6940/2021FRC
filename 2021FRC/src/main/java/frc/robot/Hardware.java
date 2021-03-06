/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.util.Constants;
import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;



public class Hardware {

    // drive motor
    public WPI_TalonFX m_leftFront;
    public WPI_TalonFX m_leftFollower;
    public WPI_TalonFX m_rghtFront;
    public WPI_TalonFX m_rghtFollower;
    public DifferentialDrive m_diffDrive;
    public Encoder m_leftEncoder;
    public Encoder m_rghtEncoder;
    public DifferentialDriveOdometry m_odometry;

    // shooter
    public WPI_TalonFX m_shooterleft;
    public WPI_TalonFX m_shooterright;
    
    // ball transport motors in front of the shooter 
    public WPI_VictorSPX m_balltransleft;
    public WPI_VictorSPX m_balltransrght;

    // intake
    public WPI_VictorSPX m_intakeleft;
    public WPI_VictorSPX  m_intakerght;
    public Solenoid m_solenoidleft;

    // navX sensor
    public AHRS m_navx;

    // color sensor
    public ColorSensorV3 m_ColorSensor; 
    public ColorMatch m_ColorMatcher;
    public WPI_TalonSRX m_colorturner;
    public Solenoid m_solenoidcolor;

    public Hardware(){

        // drive
        m_leftFront = new WPI_TalonFX(Constants.Drivebase.DRIVE_L1_PORT);
        m_leftFollower = new WPI_TalonFX(Constants.Drivebase.DRIVE_L2_PORT);
        m_rghtFront = new WPI_TalonFX(Constants.Drivebase.DRIVE_R1_PORT);
        m_rghtFollower = new WPI_TalonFX(Constants.Drivebase.DRIVE_R2_PORT);
        
        m_diffDrive = new DifferentialDrive(m_leftFront, m_rghtFront);

        m_diffDrive.setMaxOutput(Constants.Drivebase.Initial_Speed);
        m_diffDrive.setDeadband(0.05);

        m_leftEncoder = 
            new Encoder(
                Constants.DriveConstants.kLeftEncoderPorts[0],
                Constants.DriveConstants.kLeftEncoderPorts[1],
                Constants.DriveConstants.kLeftEncoderReversed);
        
        m_rghtEncoder =
            new Encoder(
                Constants.DriveConstants.kRightEncoderPorts[0],
                Constants.DriveConstants.kRightEncoderPorts[1],
                Constants.DriveConstants.kRightEncoderReversed);
        

        m_leftFront.configOpenloopRamp(Constants.Drivebase.Loop_Parameter);
        m_leftFollower.configOpenloopRamp(Constants.Drivebase.Loop_Parameter);
        m_rghtFront.configOpenloopRamp(Constants.Drivebase.Loop_Parameter);
        m_rghtFollower.configOpenloopRamp(Constants.Drivebase.Loop_Parameter);
        m_leftFront.configClosedloopRamp(Constants.Drivebase.Loop_Parameter);
        m_leftFollower.configClosedloopRamp(Constants.Drivebase.Loop_Parameter);
        m_rghtFront.configClosedloopRamp(Constants.Drivebase.Loop_Parameter);
        m_rghtFollower.configClosedloopRamp(Constants.Drivebase.Loop_Parameter);

        /* factory default values */
        //m_rghtFront.configFactoryDefault();
        //m_rghtFollower.configFactoryDefault();
        //m_leftFront.configFactoryDefault();
        //m_leftFollower.configFactoryDefault();

        /* set up followers */
        m_rghtFollower.follow(m_rghtFront);
        m_leftFollower.follow(m_leftFront);

        /* [3] flip values so robot moves forward when stick-forward/LEDs-green */
        m_rghtFront.setInverted(TalonFXInvertType.Clockwise); // !< Update this
        m_leftFront.setInverted(TalonFXInvertType.CounterClockwise); // !< Update this

        /*
         * set the invert of the followers to match their respective master controllers
         */
        m_rghtFollower.setInverted(InvertType.FollowMaster);
        m_leftFollower.setInverted(InvertType.FollowMaster);
		
        m_diffDrive.setRightSideInverted(false);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        /* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        /* config all the settings */
        m_rghtFront.configAllSettings(configs);
        m_leftFront.configAllSettings(configs);
        m_rghtFollower.configAllSettings(configs);
        m_leftFollower.configAllSettings(configs);

        // init encoders
        //m_leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,10);
        //m_rghtFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,10);

        //m_leftFront.setSensorPhase(false);
       // m_rghtFront.setSensorPhase(true);

        // reset the encoders to zero
       // m_leftFront.setSelectedSensorPosition(0,0,10);
       // m_rghtFront.setSelectedSensorPosition(0,0,10);

        // start the compressor
        //m_compressor.start();

        // shooter
        m_shooterleft = new WPI_TalonFX(Constants.shooter.Left_Shooter_Port);
        m_shooterright = new WPI_TalonFX(Constants.shooter.Right_Shooter_Port);

        m_shooterleft.setInverted(Constants.shooter.IS_LeftShooter_INVERTED);
        m_shooterright.setInverted(Constants.shooter.IS_RightShooter_INVERTED);

        // ball transport motor
        m_balltransleft = new WPI_VictorSPX(Constants.shooter.Left_Balltrans_Port);
        m_balltransrght = new WPI_VictorSPX(Constants.shooter.Right_Balltrans_Port);

        m_balltransleft.setInverted(Constants.shooter.IS_Balltransleft_INVERTED);
        m_balltransrght.setInverted(Constants.shooter.IS_BalltransRight_INVERTED);

        // intake
        m_intakeleft = new WPI_VictorSPX(Constants.Intake.Left_Intake_Port);
        m_intakerght = new WPI_VictorSPX(Constants.Intake.Right_Intake_Port);

        m_intakeleft.setInverted(Constants.Intake.Is_Intakeleft_Inverted);
        m_intakerght.setInverted(Constants.Intake.Is_Intakeright_Inverted);

        m_solenoidleft = new Solenoid(Constants.Intake.Left_Solenoid_Port);

        //navX
        m_navx = new AHRS(SPI.Port.kMXP);

        // Color Sensor
        m_ColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        m_ColorMatcher = new ColorMatch();
        m_colorturner = new WPI_TalonSRX(Constants.colorsensor.Tuner_Port);
        m_solenoidcolor = new Solenoid(Constants.colorsensor.Color_Solenoid_Port);
        
        // odometry
        m_odometry = new DifferentialDriveOdometry(m_navx.getRotation2d());
    }

    
}
