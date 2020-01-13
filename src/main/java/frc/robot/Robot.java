/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;
  public static Drivetrain drivetrain;

  
  TrajectoryConfig trajectoryConfig;
  RamseteController ramsetecontroller;
  RamseteCommand autoCommand;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    oi = new OI();
    drivetrain = new Drivetrain();
    ramsetecontroller = new RamseteController();
    trajectoryConfig  = new TrajectoryConfig(Constants.kMaxVeloctyMetersPerSecond,  Constants.kMaxAccelerationMetersPerSecondSq);
    trajectoryConfig.setKinematics(drivetrain.getDriveKinematics());
  }

  public void robotPeriodic() {
    SmartDashboard.putNumber("GYRO", drivetrain.navX.getAngle());
    SmartDashboard.putNumber("left Encoder", drivetrain.leftMaster.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Encoder", drivetrain.rightMaster.getEncoder().getPosition());
    SmartDashboard.putNumber("left Encoder2", drivetrain.leftFollower.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Encoder2", drivetrain.rightFollower.getEncoder().getPosition());


    SmartDashboard.putNumber("Right Output", drivetrain.rightMaster.getAppliedOutput());
    SmartDashboard.putNumber("Left Output", drivetrain.leftMaster.getAppliedOutput());

  }

  public void disabledInit() {
    drivetrain.leftMaster.setIdleMode(IdleMode.kCoast);
    drivetrain.rightMaster.setIdleMode(IdleMode.kCoast);
    
    drivetrain.leftFollower.setIdleMode(IdleMode.kCoast);
    drivetrain.rightFollower.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().run();
  }

  public void autonomousInit() {


    drivetrain.leftMaster.setIdleMode(IdleMode.kBrake);
    drivetrain.rightMaster.setIdleMode(IdleMode.kBrake);
    
    drivetrain.leftFollower.setIdleMode(IdleMode.kBrake);
    drivetrain.rightFollower.setIdleMode(IdleMode.kBrake);


    drivetrain.leftMaster.getEncoder().setPosition(0);
    drivetrain.rightMaster.getEncoder().setPosition(0);
    drivetrain.leftFollower.getEncoder().setPosition(0);
    drivetrain.rightFollower.getEncoder().setPosition(0);


 /*   var waypoints = Arrays.asList(
      new Pose2d(), 
      new Pose2d(1.0, 0, Rotation2d.fromDegrees(0)), 
      new Pose2d(2.0, 0, Rotation2d.fromDegrees(0))
    );
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, trajectoryConfig);
    autoCommand = new RamseteCommand( 
      trajectory,
      drivetrain::getPose,
      ramsetecontroller,
      drivetrain.getDriveFeedforward(),
      drivetrain.getDriveKinematics(),
      drivetrain::getWheelSpeeds,
      drivetrain.getLeftDriveController(),
      drivetrain.getRightDriveController(),
      drivetrain::setOutputVolts,
      drivetrain

    );
   // autoCommand.andThen(()-> drivetrain.setLeftRightMotorOutputs(0, 0));
    autoCommand.schedule();
  */

    getAutonomousCommand().schedule();
  
  }


  public Command getAutonomousCommand(){
    var waypoints = Arrays.asList(
      new Pose2d(), 
      new Pose2d(4.0, 0, Rotation2d.fromDegrees(0)), 
      new Pose2d(8.0, 0, Rotation2d.fromDegrees(0))
    );
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, trajectoryConfig);
    autoCommand = new RamseteCommand( 
      trajectory,
      drivetrain::getPose,
      ramsetecontroller,
      drivetrain.getDriveFeedforward(),
      drivetrain.getDriveKinematics(),
      drivetrain::getWheelSpeeds,
      drivetrain.getLeftDriveController(),
      drivetrain.getRightDriveController(),
      drivetrain::setOutputVolts,
      drivetrain

    );
    return autoCommand.andThen(()-> drivetrain.setLeftRightMotorOutputs(0, 0));
   
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();    
  }

  @Override
  public void teleopInit() {
    drivetrain.leftMaster.getEncoder().setPosition(0);
    drivetrain.rightMaster.getEncoder().setPosition(0);
    drivetrain.leftFollower.getEncoder().setPosition(0);
    drivetrain.rightFollower.getEncoder().setPosition(0);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted b
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    System.out.println("TEST");
 //   drivetrain.leftMaster.set(.3);
//    drivetrain.rightMaster.set(.3);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
