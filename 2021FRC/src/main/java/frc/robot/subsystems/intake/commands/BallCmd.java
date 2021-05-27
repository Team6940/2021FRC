// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.util.RobotContainer;

public class BallCmd extends CommandBase {
  /** Creates a new BallCmd. */
  public BallCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double c = 0;
    double d = 0;
    c = RobotContainer.m_driverjoystick.getTriggerAxis(GenericHID.Hand.kRight);
    d = RobotContainer.m_driverjoystick.getTriggerAxis(GenericHID.Hand.kLeft);
    SmartDashboard.putNumber("b", c);
    if(c > 0 || d > 0){
      RobotContainer.m_intake.m_intake_switch = true;
    }
    else{
      RobotContainer.m_intake.m_intake_switch = false;
    }
    RobotContainer.m_intake.OutputIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
