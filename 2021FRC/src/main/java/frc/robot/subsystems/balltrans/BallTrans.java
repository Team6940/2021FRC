// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.balltrans;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.Constants;

public class BallTrans extends SubsystemBase {
  /** Creates a new BallTrans. */
  // ball transport motor
  public WPI_VictorSPX m_balltransleft;
  public WPI_VictorSPX m_balltransrght;

  public boolean m_balltrans_switch;

  double time = Timer.getFPGATimestamp(); 

  public BallTrans() {
    m_balltransleft = Robot.hardware.m_balltransleft;
    m_balltransrght = Robot.hardware.m_balltransrght;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void BalltransWithPower(){
    m_balltransleft.set(ControlMode.PercentOutput,Constants.shooter.Left_BallTrans_On_Power);
    m_balltransrght.set(ControlMode.PercentOutput,Constants.shooter.Rght_BallTrans_On_Power); 
  }

  public void StopBalltrans(){
    m_balltransleft.set(ControlMode.PercentOutput,Constants.shooter.BallTrans_Off_Power);
    m_balltransrght.set(ControlMode.PercentOutput,Constants.shooter.BallTrans_Off_Power); 
  }

  public void OutputBalltrans(){
    if(m_balltrans_switch){
      BalltransWithPower();
    }
    else{
      StopBalltrans();
    }
  }

  public void SetBallTransBrake(){
    m_balltransleft.setNeutralMode(NeutralMode.Brake);
    m_balltransrght.setNeutralMode(NeutralMode.Brake);
  }

  public void invertballtrans(){
      m_balltransleft.set(ControlMode.PercentOutput,Constants.shooter.Balltrans_Invert_Speed);
      m_balltransrght.set(ControlMode.PercentOutput,Constants.shooter.Balltrans_Invert_Speed); 
  }
}
