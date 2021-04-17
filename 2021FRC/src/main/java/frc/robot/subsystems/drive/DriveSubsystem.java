// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;
import frc.robot.util.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; 
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
    // The robot's drive
    public WPI_TalonFX m_leftFront;
    public WPI_TalonFX m_leftFollower;
    public WPI_TalonFX m_rghtFront;
    public WPI_TalonFX m_rghtFollower;
  
    public DifferentialDrive m_diffDrive;
  
    // The left-side drive encoder
    private final Encoder m_leftEncoder =
    new Encoder(
        DriveConstants.kLeftEncoderPorts[0],
        DriveConstants.kLeftEncoderPorts[1],
        DriveConstants.kLeftEncoderReversed);
  
    // The right-side drive encoder
    private final Encoder m_rightEncoder =
    new Encoder(
        DriveConstants.kRightEncoderPorts[0],
        DriveConstants.kRightEncoderPorts[1],
        DriveConstants.kRightEncoderReversed);
  
  public DriveSubsystem() {
      // Sets the distance per pulse for the encoders
      m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
      m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_diffDrive.arcadeDrive(fwd, rot);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_diffDrive.setMaxOutput(maxOutput);
  }
}
