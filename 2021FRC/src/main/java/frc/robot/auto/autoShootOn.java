// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import frc.robot.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class autoShootOn extends CommandBase {
  int ticks = 0;
  int delayseconds;
  public autoShootOn(int seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Shooter);
    delayseconds = seconds;
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
    RobotContainer.m_Shooter.m_shooter_switch = true;
    RobotContainer.m_Shooter.OutputShooter();
    SmartDashboard.putBoolean("m_shooter_switch", RobotContainer.m_Shooter.m_shooter_switch);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // 50表示1秒
    if( ticks >= 50*delayseconds ){
       return true;
    }else {
       return false;
    }
  }
}
