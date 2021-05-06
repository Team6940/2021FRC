// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.balltrans;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.Constants;

public class BallTrans extends SubsystemBase {
  /** Creates a new BallTrans. */
  // ball transport motor
  public WPI_TalonSRX m_balltransleft;
  public WPI_TalonSRX m_balltransrght;

  public boolean m_balltrans_switch;

  public BallTrans() {
    m_balltransleft = Robot.hardware.m_balltransleft;
    m_balltransrght = Robot.hardware.m_balltransrght;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void BalltransWithPower(double ball_power){
    m_balltransleft.set(ControlMode.PercentOutput,ball_power);
    m_balltransrght.set(ControlMode.PercentOutput,ball_power);
  }

  public void OutputBalltrans(){
    if(m_balltrans_switch){
      BalltransWithPower(Constants.shooter.BallTrans_On_Power);
    }
    else{
      BalltransWithPower(Constants.shooter.BallTrans_Off_Power);
    }
  }

  public void SetBallTransBrake(){
    m_balltransleft.setNeutralMode(NeutralMode.Brake);
    m_balltransrght.setNeutralMode(NeutralMode.Brake);
  }
}
