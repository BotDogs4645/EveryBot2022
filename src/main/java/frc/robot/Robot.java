/*
  2022 everybot code
  written by carson graf 
  don't email me, @ me on discord
*/

/*
  This is catastrophically poorly written code for the sake of being easy to follow
  If you know what the word "refactor" means, you should refactor this code
*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  
  //Definitions for the hardware. 
  //update port definitions 
  CANSparkMax upperLeft = new CANSparkMax(3, MotorType.kBrushed);
  CANSparkMax lowerLeft = new CANSparkMax(2, MotorType.kBrushed);
  CANSparkMax upperRight = new CANSparkMax(5, MotorType.kBrushed);
  CANSparkMax lowerRight = new CANSparkMax(4, MotorType.kBrushed);

  CANSparkMax arm = new CANSparkMax(1, MotorType.kBrushless);
  VictorSPX intake = new VictorSPX(6);

  final MotorControllerGroup leftMotors = new MotorControllerGroup(upperLeft, lowerLeft);
  final MotorControllerGroup rightMotors = new MotorControllerGroup(upperRight, lowerRight); 

  DifferentialDrive differentialDriveSub = new DifferentialDrive(leftMotors, rightMotors);

  double leftSpeed;
  double rotSpeed;

  SlewRateLimiter speedSlew = new SlewRateLimiter(1.1);
  SlewRateLimiter rotSpeedSlew = new SlewRateLimiter(1.1);

  Joystick xbox = new Joystick(0);

  //Speed constants for controlling the arm. consider tuning these for your particular robot
  final double armHoldUp = 0.08;
  final double armHoldDown = 0.13;
  final double armTravel = 0.35;

  final double armTimeUp = 0.5;
  final double armTimeDown = 0.45;

  boolean armUp = true; //Arm initialized to up because that's how it would start a match
  boolean burstMode = false;
  double lastBurstTime = 0;

  double autoStart = 0;
  boolean goForAuto = false;

  //limelight 
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-console");

  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry limelightEnabler = table.getEntry("ledMode");

  //Limelight tracking variables
  double MIN_ROT_SPEED = .15;
  double ROT_MULTIPLIER = -0.05;
  double LIMELIGHT_HEIGHT = 3.25; // distance from ground in inches
  double GOAL_HEIGHT = 57.5; //in inches
  double LIMELIGHT_ENABLE = 0.0;
  double LIMELIGHT_DISABLE = 1.0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //Configure motors to turn correct direction. You may have to invert some of your motors
    leftMotors.setInverted(true);
    rightMotors.setInverted(false);

    limelightEnabler.setDouble(LIMELIGHT_DISABLE); // 1.0 forces the limelight off :p

    //BurnFlash: For the SPARK MAX to remember its new configuration through a power-cycle, the settings must be saved using. 
    upperLeft.burnFlash();
    lowerLeft.burnFlash();
    upperRight.burnFlash();
    lowerRight.burnFlash();
    
    arm.setInverted(false);
    //Brake Mode will effectively short all motor wires together = quick stop
    arm.setIdleMode(IdleMode.kBrake);
    arm.burnFlash();

    //add a thing on the dashboard to turn off auto if needed
    SmartDashboard.putBoolean("Go For Auto", false);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
  }

  @Override
  public void autonomousInit() {
    //get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    //check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
    limelightEnabler.setDouble(LIMELIGHT_ENABLE); // enables limeylight :p
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //arm control code. same as in teleop
    if(armUp) {
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp) {
        arm.set(armTravel);
      }
      else {
        arm.set(armHoldUp);
      }
    }
    else {
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeDown) {
        arm.set(0);
      }
      else{
        arm.set(-armHoldUp);
      }
    }

    
    //get time in seconds since start of auto
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    leftSpeed = -0.3;

    if(goForAuto){
      //series of timed events making up the flow of auto
      if(autoTimeElapsed < 3) {
        //spit out the ball for three seconds
        intake.set(ControlMode.PercentOutput, -1);
      } else if(autoTimeElapsed < 6) {
        //stop spitting out the ball and drive backwards *slowly* for three seconds
        intake.set(ControlMode.PercentOutput, 0);
        differentialDriveSub.arcadeDrive(leftSpeed, 0);
      } else if (autoTimeElapsed < 9) {
        if (tv.getDouble(0) == 0.0) {
          differentialDriveSub.arcadeDrive(0, -.3);
        } else if (tv.getDouble(0) == 1.0 && getDistance() < 6.5 ) {
          trackObject();
        }
      } else {
        //do nothing for the rest of auto
        stop();
      }
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    limelightEnabler.setDouble(2.0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() { 
    leftSpeed = xbox.getX();
    rotSpeed = xbox.getZ();
    
    
    differentialDriveSub.arcadeDrive(speedSlew.calculate(leftSpeed), rotSpeedSlew.calculate(rotSpeed));
  
    //Intake controls
    if(xbox.getRawButton(5)) {
      intake.set(VictorSPXControlMode.PercentOutput, 1);
    }
    else if(xbox.getRawButton(7)) {
      intake.set(VictorSPXControlMode.PercentOutput, -1);
    }
    else {
      intake.set(VictorSPXControlMode.PercentOutput, 0);
    }

    //Arm Controls
    if(armUp){
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp){
        arm.set(armTravel);
      }
      else{
        arm.set(armHoldUp);
      }
    }
    else{
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeDown){
        arm.set(-armTravel);
      }
      else{
        arm.set(-armHoldDown);
      }
    }
  
    if(xbox.getRawButtonPressed(6) && !armUp){
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = true;
    }
    else if(xbox.getRawButtonPressed(8) && armUp){
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = false;
    } 
    //limelight mode
    if(xbox.getRawButtonPressed(3)){
      trackObject();
    }
  }

  public void stop(){
    leftSpeed = 0;
    rotSpeed = 0;
    differentialDriveSub.arcadeDrive(leftSpeed, rotSpeed);
    arm.set(0);
    intake.set(ControlMode.PercentOutput, 0);
  }

  public void trackObject() {
    double xOffset = -tx.getDouble(0.0);
    SmartDashboard.putNumber("xOffset", xOffset);
    double finalRot = 0.0;
    if (xOffset < .25) { //0.25 represents 1/4 of a degree as measured by the limelight, this prevents the robot from overshooting its turn
      finalRot = ROT_MULTIPLIER * xOffset + MIN_ROT_SPEED;
    }
    else if (xOffset > .25) {   // dampens the rotation at the end while turning
      finalRot = ROT_MULTIPLIER * xOffset - MIN_ROT_SPEED;
    }
    differentialDriveSub.tankDrive(finalRot, -finalRot);
  }

  public double getDistance() {
    double yOffset = ty.getDouble(0.0);
    double radians = Math.toRadians(yOffset);
    double distance;
    
    distance = ((LIMELIGHT_HEIGHT - GOAL_HEIGHT) / Math.tan(radians)) / 12.0;
    SmartDashboard.putNumber("Distance to Target", distance);
    return distance;
  }

  @Override
  public void disabledInit() {
    //On disable turn off everything
    //done to solve issue with motors "remembering" previous setpoints after reenable
    stop();
    limelightEnabler.setDouble(0.0);
  }
}