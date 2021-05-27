// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.RobotContainer;
import frc.robot.util.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class autoForward extends CommandBase {
  int ticks = 0;
  int delayseconds = 0;
  double fspeed = 0;
  public autoForward(double x,int seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    delayseconds = seconds;
    fspeed = x;
    addRequirements(RobotContainer.m_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ticks = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ticks++;
    RobotContainer.m_Drive.drivemode = 0;
    RobotContainer.m_Drive.DriveCar(fspeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if( ticks >= 50*delayseconds ){
       RobotContainer.m_Drive.DriveCar(0, 0);
       RobotContainer.m_Drive.enableMotors(true);//刹车
       return true;
    }else {
       return false;
    }
  }
}
