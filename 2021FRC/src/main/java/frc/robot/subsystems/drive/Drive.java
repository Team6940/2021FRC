/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import frc.robot.Robot;

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

  public boolean auto;
  int coast =1;
  int brake = 0;

  //navX
  public AHRS m_ahrs;

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

  public void DriveCar(double x,double z){

    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("z", z);

    /* get gamepad stick values */
    double forw = -1 * x ; /* positive is forward */
    double turn = +1 * z ; /* positive is right */

    /*drive the robot*/
    m_diffDrive.arcadeDrive(forw, turn);

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
