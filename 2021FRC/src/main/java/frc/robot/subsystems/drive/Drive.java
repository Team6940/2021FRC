/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import frc.robot.Robot;
import frc.robot.util.Constants;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
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
  public Encoder m_leftEncoder;
  public Encoder m_rghtEncoder;

  public DifferentialDrive m_diffDrive;
  public DifferentialDriveOdometry m_odometry;

  public AHRS m_navx;

  //Limelight
  public NetworkTable m_limTable;

  public double tv ;
  public double ta ;
  public double tx ;
  public double ty ;

  public boolean auto = false;
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
    m_leftEncoder = Robot.hardware.m_leftEncoder;
    m_rghtEncoder = Robot.hardware.m_rghtEncoder;
  
    m_diffDrive = Robot.hardware.m_diffDrive;
    m_odometry = Robot.hardware.m_odometry;

    m_navx = Robot.hardware.m_navx;

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    m_rghtEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    
    resetEncoders();

    m_limTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_diffDrive.setMaxOutput(speed1);

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_navx.getRotation2d(), m_leftEncoder.getDistance(),
                      m_rghtEncoder.getDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double Get_tx(){
    tx = m_limTable.getEntry("tx").getDouble(0);
    SmartDashboard.putNumber("tx", tx);
    return tx;
  }

  public double Get_ty(){
    ty = m_limTable.getEntry("ty").getDouble(0);
    SmartDashboard.putNumber("ty", ty);
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
    double turn = +1 * z * 0.7 ; /* positive is right */

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

public void setFowardLittle(){
  m_diffDrive.arcadeDrive(0.05 , 0);
}

public void setBackLittle(){
  m_diffDrive.arcadeDrive(- 0.05 , 0);
}

public void setRghtLittle(){
  m_diffDrive.arcadeDrive(0 , 0.05);
}

public void setLeftLittle(){
  m_diffDrive.arcadeDrive(0 , - 0.05);
}

/**
  * Resets the drive encoders to currently read a position of 0.
  */
public void resetEncoders() {
  m_leftEncoder.reset();
  m_rghtEncoder.reset();
}

/**
  * Returns the currently-estimated pose of the robot.
  *
  * @return The pose.
  */
public Pose2d getPose() {
  return m_odometry.getPoseMeters();
}

/**
  * Returns the current wheel speeds of the robot.
  *
  * @return The current wheel speeds.
  */
public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rghtEncoder.getRate());
}

/**
  * Resets the odometry to the specified pose.
  *
  * @param pose The pose to which to set the odometry.
  */
public void resetOdometry(Pose2d pose) {
  resetEncoders();
  m_odometry.resetPosition(pose, m_navx.getRotation2d());
}

/**
  * Controls the left and right sides of the drive directly with voltages.
  *
  * @param leftVolts  the commanded left output
  * @param rightVolts the commanded right output
  */
public void tankDriveVolts(double leftVolts, double rightVolts) {
  m_leftFront.setVoltage(leftVolts);
  m_rghtFront.setVoltage(-rightVolts);
  m_diffDrive.feed();
}

/**
  * Gets the average distance of the two encoders.
  *
  * @return the average of the two encoder readings
  */
public double getAverageEncoderDistance() {
  return (m_leftEncoder.getDistance() + m_rghtEncoder.getDistance()) / 2.0;
}

/**
  * Gets the left drive encoder.
  *
  * @return the left drive encoder
  */
public Encoder getLeftEncoder() {
  return m_leftEncoder;
}

/**
  * Gets the right drive encoder.
  *
  * @return the right drive encoder
  */
public Encoder getRightEncoder() {
  return m_rghtEncoder;
}

/**
  * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
  *
  * @param maxOutput the maximum output to which the drive will be constrained
  */
public void setMaxOutput(double maxOutput) {
  m_diffDrive.setMaxOutput(maxOutput);
}

/**
  * Zeroes the heading of the robot.
  */
public void zeroHeading() {
  m_navx.reset();
}

/**
  * Returns the heading of the robot.
  *
  * @return the robot's heading in degrees, from -180 to 180
  */
public double getHeading() {
  return m_navx.getRotation2d().getDegrees();
}

/**
  * Returns the turn rate of the robot.
  *
  * @return The turn rate of the robot, in degrees per second
  */
public double getTurnRate() {
  return -m_navx.getRate();
}

}
