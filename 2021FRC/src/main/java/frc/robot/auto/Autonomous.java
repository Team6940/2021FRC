// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.util.Constants;
import frc.robot.util.RobotContainer;


public class Autonomous extends CommandBase {
  /** Creates a new Autonomous. */
  public Autonomous() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drive);
    addRequirements(RobotContainer.m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.hardware.m_leftFront.setSelectedSensorPosition(0,0,10);
    Robot.hardware.m_rghtFront.setSelectedSensorPosition(0,0,10);
    RobotContainer.m_Drive.enableMotors(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPosition = Robot.hardware.m_leftFront.getSelectedSensorPosition() * Constants.Drivebase.kDriveTick2Feet;
    double rghtPosition = Robot.hardware.m_rghtFront.getSelectedSensorPosition() * Constants.Drivebase.kDriveTick2Feet;
    double distance = (leftPosition + rghtPosition) / 2;
    double selSenVel = Robot.hardware.m_rghtFront.getSelectedSensorVelocity(0);

    
    SmartDashboard.putNumber("leftPosition", leftPosition);
    SmartDashboard.putNumber("rghtPosition", rghtPosition);
    SmartDashboard.putNumber("selSenVel", selSenVel);
    
    if(distance<1){
      RobotContainer.m_Drive.m_diffDrive.tankDrive(0.6, 0.6);
    }
    else{
      RobotContainer.m_Drive.m_diffDrive.tankDrive(0, 0);
    }
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
