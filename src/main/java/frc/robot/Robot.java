package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  CANSparkMax driveLeftA = new CANSparkMax(4, MotorType.kBrushed);
  CANSparkMax driveLeftB = new CANSparkMax(5, MotorType.kBrushed);
  CANSparkMax driveRightA = new CANSparkMax(2, MotorType.kBrushed);
  CANSparkMax driveRightB = new CANSparkMax(3, MotorType.kBrushed);

  SlewRateLimiter leftJoy = new SlewRateLimiter(2);
  SlewRateLimiter rightJoy = new SlewRateLimiter(2);

  CANSparkMax arm = new CANSparkMax(1, MotorType.kBrushless);

  VictorSPX intake = new VictorSPX(6);

  Joystick driverController = new Joystick(0);
  XboxController buttonController = new XboxController(1);

  double speed;
  double limit;

  //Constants for controlling the arm. consider tuning these for your particular robot
  final double armHoldUp = 0.11; // 0.08
  final double armHoldDown = 0.05; // 0.13
  final double armTravel = 0.4; 

  final double armTimeUp = 0.5;
  final double armTimeDown = 0.35;

  //Variables needed for the code
  boolean toggleLimit = false; // changes when button is pressed (for climbing)
  boolean armUp = true; //Arm initialized to up because that's how it would start a match FALSE?
  boolean burstMode = false;
  double lastBurstTime = 0;
  double climbStartTime = 0;

  double autoStart = 0;
  boolean goForAuto = true;

  @Override
  public void robotInit() {
    //Configure motors to turn correct direction. You may have to invert some of your motors
    driveLeftA.setInverted(true);
    driveLeftA.burnFlash();
    driveLeftB.setInverted(true);
    driveLeftB.burnFlash();
    driveRightA.setInverted(false);
    driveRightA.burnFlash();
    driveRightB.setInverted(false);
    driveRightB.burnFlash();
    
    arm.setInverted(false);
    arm.setIdleMode(IdleMode.kBrake);
    arm.burnFlash();

    toggleLimit = false;
    limit = Constants.DriveConstants.SPEED_LIMIT;

    //add a thing on the dashboard to turn off auto if needed
    SmartDashboard.putBoolean("Go For Auto", true);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", true);
  }

  @Override
  public void autonomousInit() {
    //get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    //check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", true);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    /*
    //arm control code. same as in teleop
    if(armUp){
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp){
        arm.set(armTravel); // -armTravel
      }
      else{
        arm.set(armHoldUp); // -armHoldUp
      }
    }
    else{
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeDown){
        arm.set(-armTravel); // armTravel
      }
      else{
        arm.set(-armHoldUp); // armTravel
      }
    }
    */

    //get time since start of auto
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    
    speed = Constants.DriveConstants.SPEED;

    if(goForAuto){
      //series of timed events making up the flow of auto
      if(autoTimeElapsed < 3){
        //spit out the ball for three seconds
        intake.set(ControlMode.PercentOutput, 1); // U N A B S O R B 
      }else if(autoTimeElapsed < 5){
        //stop spitting out the ball and drive backwards *slowly* for two seconds
        intake.set(ControlMode.PercentOutput, 0); // 1
        driveLeftA.set(-speed);
        driveLeftB.set(-speed);
        driveRightA.set(-speed);
        driveRightB.set(-speed);
      } else {
        //do nothing for the rest of auto
        intake.set(ControlMode.PercentOutput, 0);
        driveLeftA.set(0);
        driveLeftB.set(0);
        driveRightA.set(0);
        driveRightB.set(0);
      }
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // CLIMBING:
    if (driverController.getRawButtonPressed(Constants.ButtonConstants.TOGGLE_LIMIT)) {
      climbStartTime = Timer.getFPGATimestamp();
      if (!toggleLimit) {
        // Current state is false so turn on
        // limit = Constants.DriveConstants.CLIMB_LIMIT;
        if((Timer.getFPGATimestamp() - climbStartTime) < 0.2) {
          arm.set(armTravel);
        }
        toggleLimit = true;
      } else {
        // Current state is true so turn off
        // limit = Constants.DriveConstants.SPEED_LIMIT;
          arm.set(-0.4);
          toggleLimit = false;
        }
    }

    SmartDashboard.putBoolean("toggle", toggleLimit);
    SmartDashboard.putNumber("limit", limit);

    //Set up arcade steer
    double forward = -driverController.getY() * limit;
    double turn = -driverController.getZ() * limit;
    
    double driveLeftPower = (forward - turn);
    double driveRightPower = (forward + turn);

    SmartDashboard.putNumber("Left", driveLeftPower);
    SmartDashboard.putNumber("Right", driveRightPower);

    driveLeftA.set(leftJoy.calculate(driveLeftPower));
    driveLeftB.set(leftJoy.calculate(driveLeftPower));

    driveRightA.set(rightJoy.calculate(driveRightPower));
    driveRightB.set(rightJoy.calculate(driveRightPower));

    //Intake controls
    if(buttonController.getRawButton(Constants.ButtonConstants.INTAKE_ABSORB)) { // A B S O R B
      intake.set(VictorSPXControlMode.PercentOutput, -1); 
      SmartDashboard.putBoolean("Absorb?", true);
    }
    else if(buttonController.getRawButton(Constants.ButtonConstants.INTAKE_UNABSORB)){ // U N A B S O R B
      intake.set(VictorSPXControlMode.PercentOutput, 1); 
      SmartDashboard.putBoolean("Absorb?", false);
    }
    else{
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
        arm.set(-0.4);
      }
      else{
        arm.set(-armHoldDown);
      }
    }
  
    if(buttonController.getRawButtonPressed(Constants.ButtonConstants.ARM_UP) && !armUp) { // ARM UP
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = true;
      SmartDashboard.putBoolean("arm up", armUp);
    }
    else if(buttonController.getRawButtonPressed(Constants.ButtonConstants.ARM_DOWN) && armUp) { // ARM DOWN
      lastBurstTime = Timer.getFPGATimestamp();
      armUp = false;
      SmartDashboard.putBoolean("arm up", armUp);
    }  
  }


  @Override
  public void disabledInit() {
    driveLeftA.set(0);
    driveLeftB.set(0);
    driveRightA.set(0);
    driveRightB.set(0);
    arm.set(0);
    intake.set(ControlMode.PercentOutput, 0);
  }
}