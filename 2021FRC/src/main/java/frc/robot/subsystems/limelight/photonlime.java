// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.util.RobotContainer;

import org.photonvision.PhotonUtils;

public class photonlime extends CommandBase {
  /** Creates a new photonlime. */
  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(48.1 / 2.54);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(8) + Units.inchesToMeters(2.25);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(60);
      
  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3/0.3048);

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.28;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
  
  final double ANGULAR_P = 0.06;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  
  public photonlime() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_camera.setDriverMode(true);
    //m_camera.setLED(LEDMode.kOn);
    RobotContainer.m_Drive.auto = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed;
    double rotationSpeed;

    // Vision-alignment mode
    // Query the latest result from PhotonVision
    double target  = RobotContainer.m_Drive.Get_tv();

    if (target != 0.0f) {
        // First calculate range
        double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        CAMERA_HEIGHT_METERS,
                        TARGET_HEIGHT_METERS,
                        CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(RobotContainer.m_Drive.Get_ty()));
        SmartDashboard.putNumber("range", range);
        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort _increases_ range
        if(RobotContainer.m_Drive.Get_ty()<-1){
          forwardSpeed = 1.0 * forwardController.calculate(range, GOAL_RANGE_METERS);;
        }
        else if(RobotContainer.m_Drive.Get_ty() > 1){
          forwardSpeed = -1.0 * forwardController.calculate(range, GOAL_RANGE_METERS);
        }
        else{
          forwardSpeed = 0;
        }


        // Also calculate angular power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        rotationSpeed = -1.0 * turnController.calculate(RobotContainer.m_Drive.Get_tx(), 0);
      } else {
          // If we have no targets, stay still.
          forwardSpeed = 0;
          rotationSpeed = 0;
        } 
      // Use our forward/turn speeds to control the drivetrain
      Robot.hardware.m_diffDrive.arcadeDrive(forwardSpeed,rotationSpeed);
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
