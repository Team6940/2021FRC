// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.Constants;
import frc.robot.util.RobotContainer;

public class Shooter extends SubsystemBase {
  // motor
  public WPI_TalonFX m_shooterleft;
  public WPI_TalonFX m_shooterright;

  public boolean m_switch;

  // Current Distance between robot and the target
  public static double d;

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterleft = Robot.hardware.m_shooterleft;
    m_shooterright = Robot.hardware.m_shooterright;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ShooterWithPower(double power){
    m_shooterleft.set(ControlMode.PercentOutput,power);
    m_shooterright.set(ControlMode.PercentOutput,power);
  }

  public void OutputShooter(){
    if(m_switch){
      ShooterWithPower(Constants.shooter.ShooterON_Percent_Output);
    }
    else{
      ShooterWithPower(Constants.shooter.ShooterOff_Percent_Output);
    }
  }

  public void Auto_Collimation(){
    d = (Constants.Limelight.Target_Height - Constants.Limelight.Shooter_Height) / Math.tan(Constants.Limelight.Limelight_Angle + RobotContainer.m_Drive.Get_ty());
    SmartDashboard.putNumber("d", d);
  }


  public void SetShooterBrake(){
    m_shooterleft.setNeutralMode(NeutralMode.Brake);
    m_shooterright.setNeutralMode(NeutralMode.Brake);
  }
}
