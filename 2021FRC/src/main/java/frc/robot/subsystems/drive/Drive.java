/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import frc.robot.Robot;
import frc.robot.util.Constants;
import frc.robot.util.RobotContainer;

import com.kauailabs.navx.frc.AHRS;

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

  public double tv ;
  public double ta ;
  public double tx ;
  public double ty ;

  boolean m_LimelightHasValidTarget = false;
  double m_LimelightforwCommand = 0.0;
  double m_LimelightturnCommand = 0.0;

  public boolean auto;
  int coast =1;
  int brake = 0;

  //navX
  public AHRS m_ahrs;
  public boolean autoBalanceXMode;
  public boolean autoBalanceYMode;

  //Speed threshold
  double speed1 =  0.5;

  public Drive() {
    //here is todo
    m_leftFront = Robot.hardware.m_leftFront;
    m_leftFollower = Robot.hardware.m_leftFollower;
    m_rghtFront = Robot.hardware.m_rghtFront;
    m_rghtFollower = Robot.hardware.m_rghtFollower;
  
    m_diffDrive = Robot.hardware.m_diffDrive;
    m_limTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_diffDrive.setMaxOutput(speed1);

    //navX
    m_ahrs = Robot.hardware.ahrs;
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
    double Left_Threshold = Constants.Limelight.StopLime_ThresholdLeft;
    double Rght_Threshold = Constants.Limelight.StopLime_ThresholdRght;
    //double forw_cmd;

    //Show the newtworktable number
    SmartDashboard.putNumber("tv", Get_tv());
    SmartDashboard.putNumber("ta", Get_ta());
    SmartDashboard.putNumber("tx", Get_tx());
    SmartDashboard.putNumber("ty", Get_ty());

  
    if (Get_tv() < 1.0){
      m_LimelightHasValidTarget = false;
      m_LimelightforwCommand = 0.0;
      m_LimelightturnCommand = 0.0;
      return;
    }
  
    m_LimelightHasValidTarget = true;
  
    // Start with proportional steering
    if(Get_tx() > Left_Threshold && Get_tx()< Rght_Threshold){
      turn_cmd = 0;
      m_LimelightturnCommand = turn_cmd;
    }
    else{
      turn_cmd = Get_tx() * Constants.Limelight.STEER_K;
      m_LimelightturnCommand = turn_cmd;
    }
  }

  public double Get_tx(){
    tx = m_limTable.getEntry("tx").getDouble(0);
    return tx;
  }

  public double Get_ty(){
    ty = m_limTable.getEntry("ty").getDouble(0);
    return ty;
  }

  public double Get_ta(){
    ta = m_limTable.getEntry("ta").getDouble(0);
    return ta;
  }

  public double Get_tv(){
    tv = m_limTable.getEntry("tv").getDouble(0);
    return tv;
  }



  public void DriveCar(double x,double z,boolean qt){

    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("z", z);

    Update_Limelight_Tracking();

    /* get gamepad stick values */
    double forw = -1 * x ; /* positive is forward */
    double turn = +1 * z ; /* positive is right */

    /* get necessary parameters for AutoBalance*/
    double xAxisRate            = RobotContainer.m_stickL.getX();
    double yAxisRate            = RobotContainer.m_stickL.getY();
    double pitchAngleDegrees    = m_ahrs.getPitch();
    double rollAngleDegrees     = m_ahrs.getRoll();

    // AutoBalance
    if ( !autoBalanceXMode && 
      (Math.abs(pitchAngleDegrees) >= 
      Math.abs(Constants.navX.kOffBalanceAngleThresholdDegrees))) {
      autoBalanceXMode = true;
    }
    else if ( autoBalanceXMode && 
          (Math.abs(pitchAngleDegrees) <= 
            Math.abs(Constants.navX.kOonBalanceAngleThresholdDegrees))) {
      autoBalanceXMode = false;
    }
    if ( !autoBalanceYMode && 
      (Math.abs(pitchAngleDegrees) >= 
      Math.abs(Constants.navX.kOffBalanceAngleThresholdDegrees))) {
      autoBalanceYMode = true;
    }
    else if ( autoBalanceYMode && 
          (Math.abs(pitchAngleDegrees) <= 
            Math.abs(Constants.navX.kOonBalanceAngleThresholdDegrees))) {
    autoBalanceYMode = false;
    }
  
    // Control drive system automatically, 
    // driving in reverse direction of pitch/roll angle,
    // with a magnitude based upon the angle
  
    if ( autoBalanceXMode ) {
      double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
      xAxisRate = Math.sin(pitchAngleRadians) * -1;
    }
    if ( autoBalanceYMode ) {
      double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
      yAxisRate = Math.sin(rollAngleRadians) * -1;
    }


   /*drive the robot*/
    if (auto){
      if (m_LimelightHasValidTarget){
        //setLightMode(Constants.Limelight.LED_ON);
        m_diffDrive.arcadeDrive(m_LimelightforwCommand,m_LimelightturnCommand);
      }
      else{
        m_diffDrive.arcadeDrive(0.0,0.0);
        //setLightMode(Constants.Limelight.LED_OFF);
      }
    }
    else{
      if(autoBalanceYMode){
        if(qt){
          m_diffDrive.curvatureDrive(xAxisRate, yAxisRate, qt);
          SmartDashboard.putNumber("xAxisRate", xAxisRate);
          SmartDashboard.putNumber("yAxisRate", yAxisRate);
        }
        else{
          m_diffDrive.arcadeDrive(xAxisRate, yAxisRate);
          SmartDashboard.putNumber("xAxisRate", xAxisRate);
          SmartDashboard.putNumber("yAxisRate", yAxisRate);
        }
      }
      else{
        if(qt){
          m_diffDrive.curvatureDrive(forw, turn, qt);
          //setCoast();
        }
        else{
          m_diffDrive.arcadeDrive(forw, turn);
          //setCoast();
          SmartDashboard.putNumber("turn_cmd",m_LimelightturnCommand);
        }
      }
    }

     
   /* m_diffDrive.arcadeDrive(m_joystick.getRawAxis(1), -m_joystick.getRawAxis(0));
   m_diffDrive.arcadeDrive(m_joystick.getRawAxis(1), -m_joystick.getRawAxis(0));*/
}

public void AutoBalanceinit(){ 
}

public void AutoBalance(){

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

public void enableMotors(boolean on){
  NeutralMode mode;
  if(on){
    mode = NeutralMode.Brake;
  }
  else{
    mode = NeutralMode.Coast;
  }
  m_leftFront.setNeutralMode(mode);
  m_leftFollower.setNeutralMode(mode);
  m_rghtFront.setNeutralMode(mode);
  m_rghtFollower.setNeutralMode(mode);
  //coast++;
}
}
