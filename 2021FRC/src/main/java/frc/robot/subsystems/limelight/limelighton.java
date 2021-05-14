// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.util.Constants;
import frc.robot.util.RobotContainer;

public class limelighton extends CommandBase {
  static double test = 0;
  /** Creates a new limelight. */
  public limelighton() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get tx 
    double lime_tx = RobotContainer.m_Drive.Get_tx();

    //  Set the parameters
    double heading_error = - RobotContainer.m_Drive.Get_tx();
    // You need to reset the cross_hair in order to use this method . Unless you will need to calculate the current distance.
    double distance_error = - RobotContainer.m_Drive.Get_ty(); 

    // Package the parameters
    double KpAim = Constants.Limelight.KpAim;
    double KpDistance = Constants.Limelight.KpDistance;

    // Package the degree of drive
    double min_command = Constants.Limelight.min_command;

    // Set some crucial parameters
    double steering_adjust = 0.0;
    double left_command = 0.0;
    double rght_command = 0.0;
    double distance_adjust = 0.0;

    if(lime_tx>1.0){
      steering_adjust = KpAim * heading_error - min_command;
    }
    else if(lime_tx < 1.0){
      steering_adjust = KpAim * heading_error + min_command;
    }

    distance_adjust = KpDistance * distance_error;

    left_command += steering_adjust + distance_adjust;
    rght_command += steering_adjust + distance_adjust;

    Robot.hardware.m_diffDrive.tankDrive(left_command, rght_command);
    test ++;
    SmartDashboard.putNumber("test",test);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
