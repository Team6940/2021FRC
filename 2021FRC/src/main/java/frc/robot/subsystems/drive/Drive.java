/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import frc.robot.Robot;
import frc.robot.util.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; 

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Drive extends SubsystemBase {

  //motor
  public WPI_TalonFX m_leftFront;
  public WPI_TalonFX m_leftFollower;
  public WPI_TalonFX m_rghtFront;
  public WPI_TalonFX m_rghtFollower;

  public DifferentialDrive m_diffDrive;


  //Limelight
  public NetworkTable m_limTable;

  public double tx;
  public double ty;
  public double ta;
  public double tv;

  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightforwCommand = 0.0;
  private double m_LimelightturnCommand = 0.0;

  public boolean auto;

  //Speed threshold
  private double speed1 = 0.5;

  public Drive() {
    //here is todo
    m_leftFront = Robot.hardware.m_leftFront;
    m_leftFollower = Robot.hardware.m_leftFollower;
    m_rghtFront = Robot.hardware.m_rghtFront;
    m_rghtFollower = Robot.hardware.m_rghtFollower;
  
    m_diffDrive = Robot.hardware.m_diffDrive;
    m_limTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_diffDrive.setMaxOutput(speed1);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void Update_Limelight_Tracking(){
    double turn_cmd;
    double forw_cmd;

    tv=m_limTable.getEntry("tv").getDouble(0);
    ta=m_limTable.getEntry("ta").getDouble(0);
    tx=m_limTable.getEntry("tx").getDouble(0);
    ty=m_limTable.getEntry("ty").getDouble(0);

    //Show the newtworktable number
    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("ta", ta);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);

  
    if (tv < 1.0){
      m_LimelightHasValidTarget = false;
      m_LimelightforwCommand = 0.0;
      m_LimelightturnCommand = 0.0;
      return;
    }
  
    m_LimelightHasValidTarget = true;
  
    // Start with proportional steering
    turn_cmd = tx * Constants.Limelight.STEER_K;
    m_LimelightturnCommand = turn_cmd;

    // try to drive forward until the target area reaches our desired area
    forw_cmd = (Constants.Limelight.DESIRED_TARGET_AREA - ta) * Constants.Limelight.DRIVE_K;
    
    // don't let the robot drive too fast into the goal
    if (forw_cmd > Constants.Limelight.MAX_DRIVE){
      forw_cmd = Constants.Limelight.MAX_DRIVE;
    }
    m_LimelightforwCommand = forw_cmd;
  
  }

  public void DriveCar(double x,double z,boolean qt){

    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("z", z);

    Update_Limelight_Tracking();

    /* get gamepad stick values */
    double forw = -1 * x; /* positive is forward */
    double turn = +1 * z; /* positive is right */


   /* If you want to set advanced speed , add speed1 or speed2 in the front*/

   /* deadband gamepad 10% */
    if (Math.abs(forw) < 0.10) {
       forw = 0;
    }
    if (Math.abs(turn) < 0.10) {
       turn = 0;
    }

   /*drive the robot*/
    if (auto){
      if (m_LimelightHasValidTarget){
        setLightMode(Constants.Limelight.LED_FLASH);
        m_diffDrive.arcadeDrive(m_LimelightforwCommand,m_LimelightturnCommand);
      }
      else{
        m_diffDrive.arcadeDrive(0.0,0.0);
      }
    }
    else{
      if(qt){
        m_diffDrive.curvatureDrive(forw, turn, qt);
      }
      else{
        m_diffDrive.arcadeDrive(forw, turn);
      }
    }

     
   /* m_diffDrive.arcadeDrive(m_joystick.getRawAxis(1), -m_joystick.getRawAxis(0));
   m_diffDrive.arcadeDrive(m_joystick.getRawAxis(1), -m_joystick.getRawAxis(0));*/
}

public void setLightMode(int mode){
  m_limTable.getEntry("ledMode").setNumber(mode);
}

public void setFast(){
    speed1 += 0.1;

    speed1 = MathUtil.clamp(speed1,-1.0,1.0);
    m_diffDrive.setMaxOutput(speed1);
   
}

public void setSlow(){
      speed1 -= 0.1;
      speed1 = MathUtil.clamp(speed1,0,1.0);
      m_diffDrive.setMaxOutput(speed1);
}
public void setBrake(){
        m_leftFront.setNeutralMode(NeutralMode.Brake);
        m_leftFollower.setNeutralMode(NeutralMode.Brake);
        m_rghtFront.setNeutralMode(NeutralMode.Brake);
        m_rghtFollower.setNeutralMode(NeutralMode.Brake);
}
public void setCoast(){
        m_leftFront.setNeutralMode(NeutralMode.Coast);
        m_leftFollower.setNeutralMode(NeutralMode.Coast);
        m_rghtFront.setNeutralMode(NeutralMode.Coast);
        m_rghtFollower.setNeutralMode(NeutralMode.Coast);
      }
}
