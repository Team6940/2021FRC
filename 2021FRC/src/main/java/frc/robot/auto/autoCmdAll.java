// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import frc.robot.util.RobotContainer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.FollowTrajectoryCommand;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.util.Constants;

/** An example command that uses an example subsystem. */
public class autoCmdAll extends SequentialCommandGroup {

  /**
   * Creates a new DriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public autoCmdAll(){
    //step1.auto intake three balls from LOADING BAY.  2S : 不需要了，因球已经提前装到机器中
    //step2.auto drive to POWER PORT using Trajectory1 5S
    //step3.auto shoot three balls using limelight arm  2S
    //step4.auto drive to TRENCH RUN area using Trajectory2 and intake three balls using parall cmd
    //step5.auto drive to POWER PORT using Trajectory3
    //step6.auto shoot three balls using limelight arm  2S
    //step7.auto drive back to INITIATION LINE.
    //step8.stop all motors.
    Trajectory Trajectory1 = RobotContainer.m_trajectories.get("intakeThreeBalls");
    Trajectory Trajectory2 = RobotContainer.m_trajectories.get("straightline");
    // RobotContainer.getDefaultTrajectory 获取的固定路径，如果需要编辑请去定义处手动修改
    Trajectory Trajectory3 = RobotContainer.m_trajectory;
    RamseteCommand ramseteCommand =
        new RamseteCommand(
          Trajectory3,
          RobotContainer.m_Drive::getPose,
            new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DriveConstants.kDriveKinematics,
            RobotContainer.m_Drive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            RobotContainer.m_Drive::tankDriveVolts,
            RobotContainer.m_Drive);
    
    addCommands (
        new autoShootOn(2), 
        new autoBallTransOn(0),
        new ParallelCommandGroup(  //三个命令并行执行
          new autoShootOn(3), // 参数表示命令执行延时多少秒
          new autoBallTransOn(3),// 参数表示命令执行延时多少秒
          new autoIntakeOn(3)// 参数表示命令执行延时多少秒
          ),
        new autoShooterStop(0),
              //ramseteCommand.andThen(() -> RobotContainer.m_Drive.tankDriveVolts(0, 0))
        new autoForward(10 ,3)
               //new FollowTrajectoryCommand(Trajectory2, RobotContainer.m_Drive)
            
            );
        
  }


}
