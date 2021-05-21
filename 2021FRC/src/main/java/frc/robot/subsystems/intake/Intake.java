// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  // motor
  public WPI_VictorSPX m_intakeleft;
  public WPI_VictorSPX m_intakerght;

  // Solenoid
  public Solenoid m_solenoidleft;
  public Solenoid m_solenoidrght;

  public boolean m_intake_switch = false;

  public Intake() {
    m_intakeleft = Robot.hardware.m_intakeleft;
    m_intakerght = Robot.hardware.m_intakerght;
    m_solenoidleft = Robot.hardware.m_solenoidleft;
    m_solenoidrght = Robot.hardware.m_solenoidrght;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SolenoidWithSwitch(boolean on){
    m_solenoidleft.set(on);
    m_solenoidrght.set(on);
  }

  public void IntakeWithPower(double Intake_Power){
    m_intakeleft.set(ControlMode.PercentOutput,Intake_Power);
    m_intakerght.set(ControlMode.PercentOutput,Intake_Power);
  }

  public void OutputIntake(){
    if(m_intake_switch){
      IntakeWithPower(Constants.Intake.Intake_Start_Speed);
    }
    else{
      IntakeWithPower(Constants.Intake.Intake_Stop_Speed);
    }
  }
}
