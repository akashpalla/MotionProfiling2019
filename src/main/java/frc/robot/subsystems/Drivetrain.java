/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Add your docs here.
 */
public class Drivetrain extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  public TalonFX leftMaster, rightMaster, leftFollower, rightFollower;
  public AHRS navX;

  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;

  private SimpleMotorFeedforward feedForward; 
  private PIDController leftDriveController, rightDriveController;
 
  public Drivetrain(){
    leftMaster = new TalonFX(5);
    rightMaster = new TalonFX(1);
    leftFollower =new TalonFX(3);
    rightFollower = new TalonFX(2);
  
    leftMaster.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightFollower.configFactoryDefault();

    navX = new AHRS(SPI.Port.kMXP);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);
    rightFollower.setInverted(true);
    leftFollower.setInverted(false);


   
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);


    resetEncoderPosition();
  
    navX.reset();

    kinematics = new DifferentialDriveKinematics(Constants.kTrackWidthMeters);
    odometry = new DifferentialDriveOdometry(getGyroAngle());
    feedForward = new SimpleMotorFeedforward(0.319,5.2,.352);  // Use Gains from Characteriztion Tool
    //feedForward = new SimpleMotorFeedforward(0, 3,0);
    leftDriveController = new PIDController(0, 0, 0);   // Use Gains from Characterization Tool
    rightDriveController = new PIDController(0, 0, 0);
   
    
  }

  @Override
  public void periodic() {    
    odometry.update(getGyroAngle(),getLeftEncoderDistance(), getRightEncoderDistance());

    Pose2d curr = odometry.getPoseMeters();

    SmartDashboard.putNumber("X", curr.getTranslation().getX());
    SmartDashboard.putNumber("Y", curr.getTranslation().getY());

   
  }


  public void configureIdleMode(NeutralMode mode){
    leftMaster.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    leftFollower.setNeutralMode(mode);
    rightFollower.setNeutralMode(mode);
  }

  public void resetEncoderPosition(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
    leftFollower.setSelectedSensorPosition(0);
    rightFollower.setSelectedSensorPosition(0);
  }

  public void cheesyIshDrive(double throttle, double wheel, boolean quickTurn){
    double leftOutput, rightOutput = 0;
    
    //Deadband
    if (epsilonEquals(throttle, 0.0, 0.04)) {
      throttle = 0.0;
    }

    if (epsilonEquals(wheel, 0.0, 0.035)) {
      wheel = 0.0;
   }

    
    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= kWheelGain;
    Twist2d motion = new Twist2d(throttle, 0, wheel);
    if (Math.abs(motion.dtheta) < Constants.kEpsilon) {
      leftOutput = motion.dx;
      rightOutput = motion.dx;
    }else{
      double delta_v = Units.metersToInches(Constants.kTrackWidthMeters) * motion.dtheta / (2 * Constants.kTrackScrubFactor);
      leftOutput = motion.dx - delta_v;
      rightOutput = motion.dx + delta_v;
    }

    //Normalize the Output on a -1 to 1 Scale
    double scaling_factor = Math.max(1.0, Math.max(Math.abs(leftOutput), Math.abs(rightOutput)));
    setLeftRightMotorOutputs(leftOutput/scaling_factor, rightOutput/scaling_factor);
  }

  public void setLeftRightMotorOutputs(double left, double right){
    leftMaster.set(ControlMode.PercentOutput, left);
    rightMaster.set(ControlMode.PercentOutput, right);

  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
}

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
  
  public Rotation2d getGyroAngle(){
    return Rotation2d.fromDegrees(-navX.getAngle());
  }

  public void resetOdometry(){
    resetEncoderPosition();
    navX.reset();
    odometry.resetPosition(new Pose2d(), getGyroAngle());
  }

  //METERS
  public double getLeftEncoderDistance(){
   // System.out.println(leftMaster.getEncoder().getPosition() / Constants.kDriveGearRatio * 2 * Math.PI * Constants.kWheelRadiusMeters);
    return leftMaster.getSelectedSensorPosition() / 2048 / Constants.kDriveGearRatio * 2 * Math.PI * Constants.kWheelRadiusMeters;

  }

  //METERS
  public double getRightEncoderDistance(){
    return rightMaster.getSelectedSensorPosition() / 2048 / Constants.kDriveGearRatio * 2 * Math.PI * Constants.kWheelRadiusMeters;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      leftMaster.getSelectedSensorVelocity() / 2048 /Constants.kDriveGearRatio * Constants.kWheelCircumferenceMeters * 10,
      rightMaster.getSelectedSensorVelocity() / 2048 /Constants.kDriveGearRatio * Constants.kWheelCircumferenceMeters* 10
    );
  }

  public void setOutputVolts(double leftVolts, double rightVolts){
    setLeftRightMotorOutputs(leftVolts/12, rightVolts/12);
  }


  public PIDController getLeftDriveController(){
    return leftDriveController;
  }

  public PIDController getRightDriveController(){
    return rightDriveController;
  }

  public SimpleMotorFeedforward getDriveFeedforward(){
    return feedForward;
  }

  public DifferentialDriveKinematics getDriveKinematics(){
    return kinematics;
  }


  public void log(){
    SmartDashboard.putNumber("LEFT Distance Meters", getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoder Meters", getRightEncoderDistance());
  }
  


}
