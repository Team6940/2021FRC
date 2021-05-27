// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.RobotContainer;
import frc.robot.util.Constants;

public class autoSolenoidStart extends CommandBase {
  double starttime1,starttime2 ; 
  boolean flag = false;
  
  public autoSolenoidStart() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    starttime1 = RobotContainer.m_timer.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_intake.SolenoidWithSwitch(Constants.Intake.Soenoid_Start);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double time = RobotContainer.m_timer.get();
    if( (time - starttime1) >= 3.0f ){
       return true;
    }else {
       return false;
    }
  }
}
