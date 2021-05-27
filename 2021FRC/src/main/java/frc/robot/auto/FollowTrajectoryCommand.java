package frc.robot.auto;

import frc.robot.util.Constants;
import frc.robot.subsystems.drive.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class FollowTrajectoryCommand extends SequentialCommandGroup {

    public FollowTrajectoryCommand(Trajectory trajectory,Drive drivesystem) {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(
              Constants.DriveConstants.ksVolts,
              Constants.DriveConstants.kvVoltSecondsPerMeter,
              Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
          Constants.DriveConstants.kDriveKinematics,
          10);
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
              Constants.AutoConstants.kMaxSpeedMetersPerSecond,
              Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.DriveConstants.kDriveKinematics)
          // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    addCommands(
      new RamseteCommand(
        trajectory,
        drivesystem::getPose,
          new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(
              Constants.DriveConstants.ksVolts,
              Constants.DriveConstants.kvVoltSecondsPerMeter,
              Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
          Constants.DriveConstants.kDriveKinematics,
          drivesystem::getWheelSpeeds,
          new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
          new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          drivesystem::tankDriveVolts,
          drivesystem),
          new InstantCommand(() -> drivesystem.tankDriveVolts(0, 0)));
    
    }

    @Override
    public void initialize() {
     
    }
}
