// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.util.Units;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  Joystick operatorJoystick = new Joystick(2);
  Joystick leftJoystick = new Joystick(1);
  Joystick rightJoystick = new Joystick(0);
  
  Spark driveMotor1 = new Spark(0);
  Spark driveMotor2 = new Spark(1);
  Spark driveMotor3 = new Spark(2);
  Spark driveMotor4 = new Spark(3);

  MotorControllerGroup leftDriveMoters = new MotorControllerGroup(driveMotor1, driveMotor2);
  MotorControllerGroup rightDriveMoters = new MotorControllerGroup(driveMotor3, driveMotor4);

  DifferentialDrive driveTrain = new DifferentialDrive(leftDriveMoters, rightDriveMoters);

  PhotonCamera camera = new PhotonCamera("Logitech,_Inc._Webcam_C310");
  PhotonPipelineResult result; PhotonTrackedTarget target;
  Double yaw; Double pitch; int aprilTagID; 

  final double camera_height_meters = Units.inchesToMeters(6.25);
  final double target_height_meters = Units.feetToMeters(1.225);
  final double camera_pitch_radians = Units.degreesToRadians(20);

  double range;
  double restrictValue = 1;
  boolean restrict = false;

  PneumaticsControlModule pcm = new PneumaticsControlModule(0);
	DoubleSolenoid piston_1 = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 4, 5);

  private final AnalogInput ultrasonic = new AnalogInput(0);
	private final DigitalOutput ultrasonicPin_1 = new DigitalOutput(9);
  short currentDistanceInches;

  Thread operatorThread; Boolean isOnOperator = true; 
  
  int kX1 = 1; int kX2 = 1;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    leftDriveMoters.setInverted(true);
    pcm.enableCompressorDigital();
    //pcm.enableCompressorAnalog(kDefaultPeriod, x);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    
   //driveTrain.tankDrive(0.7, -0.7);

    //Timer.delay(5);
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    camera.setPipelineIndex(2);
    ultrasonicPin_1.set(true);
    
  }

  /** This function is called periodically during autonomous. */
  int i = 0;
  int maxValue = 200000;
  @Override
  public void autonomousPeriodic() {
    while (i != maxValue) {
    driveTrain.tankDrive(0.7, -0.7);
    i++;
  }
  if (i == maxValue){
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        if (kX2 == 1) {
          kX2 = goBackwards();
          System.out.println("Entered Go Backwards");
        } if (kX1 > kX2) {
            kX1 = goToAprilTag(range);
            System.out.println("Entered Go To Apriltag");
        }
      }
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    isOnOperator = true;
    operatorThread = new Thread(() -> {
      while (isOnOperator) {
        //Put Arm Code Here
        if (leftJoystick.getRawButton(6)) {
          piston_1.set(Value.kForward);
        } else if (leftJoystick.getRawButton(5)) {
          piston_1.set(Value.kReverse);
        } else {
          piston_1.set(Value.kOff);
        }
      }
    });
    operatorThread.setDaemon(true);
    operatorThread.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    

    /*if(rightJoystick.getRawButton(3) && !restrict) {
      restrict = true;
      restrictValue = 0.25;
      System.out.println("restricted");
    } else if (rightJoystick.getRawButton(3) && restrict) {
      restrict = false;
      restrictValue = 1;
      System.out.println("not restricted");
    }*/

    restrictValue = 0.8;

    driveTrain.tankDrive(leftJoystick.getY() * restrictValue, rightJoystick.getY() * restrictValue);

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    isOnOperator = false;
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public int goToAprilTag(double range) {
    result = camera.getLatestResult();
      if (result.hasTargets()) {
        range = Units.metersToFeet(PhotonUtils.calculateDistanceToTargetMeters(camera_height_meters,target_height_meters,camera_pitch_radians,Units.degreesToRadians(result.getBestTarget().getPitch())));
        System.out.println("The distance to the target in feet is: " + range);
        if (camera.getPipelineIndex() == 0) {
          target = result.getBestTarget();
          yaw = target.getYaw();
          pitch = target.getPitch();
          System.out.println("Yaw: " + yaw + " / Pitch: " + pitch);
        } else if (camera.getPipelineIndex() == 2) {
            target = result.getBestTarget();
            yaw = target.getYaw();
            pitch = target.getPitch();
            aprilTagID = target.getFiducialId();
            System.out.println("Yaw: " + yaw + " / Pitch: " + pitch + " AprilTagID: " + aprilTagID);
        } if (range > 1) {
            if (yaw <= -6.667) {
              driveTrain.tankDrive(-0.20, 0.20);
            } else if (yaw > -6.667 && yaw <= 6.666) {
              driveTrain.tankDrive(0.20, 0.20);
            } else if (yaw > 6.666 && yaw <= 30.0) {
              driveTrain.tankDrive(0.20, -0.20);
          }
        } else {
          driveTrain.tankDrive(0, 0);
          return 0;
        }
      } else {
        driveTrain.tankDrive(0, 0);
        return 1;
    }
    return 1;
  }

  public int goBackwards() {
    while (kX2 == 1) {
      currentDistanceInches = (short) Math.round(Distance.distCalc(ultrasonic.getValue()));
      System.out.println("Object distance in inches: " + currentDistanceInches);
      if (currentDistanceInches > 30) {
        driveTrain.tankDrive(-0.5, -0.5);
      } else {
        driveTrain.tankDrive(0.5, 0.5);
        for(int i = 0; i < 25000;) {}
        driveTrain.tankDrive(0,0);
        return 0;
      }
    }
    return 0;
  }
}
