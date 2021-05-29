// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class autoIntakeOn extends CommandBase {
  int ticks = 0;
  int delayseconds = 0;
  public autoIntakeOn(int seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    delayseconds = seconds;
    addRequirements(RobotContainer.m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ticks = 0;
    RobotContainer.m_colorsensor.colorsensorsolenoid(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ticks++;
    RobotContainer.m_intake.m_intake_switch = true;
    //RobotContainer.m_intake.OutputIntake();
    SmartDashboard.putBoolean("m_intake_switch",RobotContainer.m_intake.m_intake_switch);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_colorsensor.colorsensorsolenoid(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if( ticks >= 50*delayseconds ){
       return true;
    }else {
       return false;
    }
  }
}
