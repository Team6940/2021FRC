// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  // motor
  public WPI_TalonSRX m_intakeleft;
  public WPI_TalonSRX m_intakerght;

  // Solenoid
  public Solenoid m_solenoidleft;
  public Solenoid m_solenoidrght;

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

}
