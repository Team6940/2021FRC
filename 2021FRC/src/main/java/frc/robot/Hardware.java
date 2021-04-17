/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.util.Constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;



public class Hardware {

    // drive motor
    public WPI_TalonFX m_leftFront;
    public WPI_TalonFX m_leftFollower;
    public WPI_TalonFX m_rghtFront;
    public WPI_TalonFX m_rghtFollower;

    public DifferentialDrive m_diffDrive;

    // shooter
    public WPI_TalonFX m_shooterleft;
    public WPI_TalonFX m_shooterright;

    // navX sensor
    public AHRS ahrs;

    public Hardware(){

        // drive
        m_leftFront = new WPI_TalonFX(Constants.Drivebase.DRIVE_L1_PORT);
        m_leftFollower = new WPI_TalonFX(Constants.Drivebase.DRIVE_L2_PORT);
        m_rghtFront = new WPI_TalonFX(Constants.Drivebase.DRIVE_R1_PORT);
        m_rghtFollower = new WPI_TalonFX(Constants.Drivebase.DRIVE_R2_PORT);
        
        m_diffDrive = new DifferentialDrive(m_leftFront, m_rghtFront);

        m_diffDrive.setMaxOutput(Constants.Drivebase.Initial_Speed);
        
        // set "soft boot"
        
        m_leftFront.configOpenloopRamp(Constants.Drivebase.Loop_Parameter);
        m_leftFollower.configOpenloopRamp(Constants.Drivebase.Loop_Parameter);
        m_rghtFront.configOpenloopRamp(Constants.Drivebase.Loop_Parameter);
        m_rghtFollower.configOpenloopRamp(Constants.Drivebase.Loop_Parameter);
        m_leftFront.configClosedloopRamp(Constants.Drivebase.Loop_Parameter);
        m_leftFollower.configClosedloopRamp(Constants.Drivebase.Loop_Parameter);
        m_rghtFront.configClosedloopRamp(Constants.Drivebase.Loop_Parameter);
        m_rghtFollower.configClosedloopRamp(Constants.Drivebase.Loop_Parameter);
      
        /* factory default values */
        m_rghtFront.configFactoryDefault();
        m_rghtFollower.configFactoryDefault();
        m_leftFront.configFactoryDefault();
        m_leftFollower.configFactoryDefault();

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

        // shooter
        m_shooterleft = new WPI_TalonFX(Constants.shooter.Left_Shooter_Port);
        m_shooterright = new WPI_TalonFX(Constants.shooter.Right_Shooter_Port);

        m_shooterleft.setInverted(Constants.shooter.IS_LeftShooter_INVERTED);
        m_shooterright.setInverted(Constants.shooter.IS_RightShooter_INVERTED);

        //navX
        ahrs = new AHRS(SPI.Port.kMXP);

    }

    
}
