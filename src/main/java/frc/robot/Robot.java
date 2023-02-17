// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PneumaticsControlModule;

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

  Spark armMotor1 = new Spark(4);

  MotorControllerGroup leftDriveMoters = new MotorControllerGroup(driveMotor1, driveMotor2);
  MotorControllerGroup rightDriveMoters = new MotorControllerGroup(driveMotor3, driveMotor4);

  DifferentialDrive driveTrain = new DifferentialDrive(leftDriveMoters, rightDriveMoters);

  PneumaticsControlModule pcm = new PneumaticsControlModule(0);
	DoubleSolenoid piston_1 = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 4, 5);

  PhotonCamera camera = new PhotonCamera("Logitech,_Inc._Webcam_C310");
  PhotonPipelineResult result; PhotonTrackedTarget target;

  Double yaw; Double pitch; int aprilTagID;

  private final AnalogInput ultrasonic = new AnalogInput(0);

	private final DigitalOutput ultrasonicPin_1 = new DigitalOutput(0);

	short currentDistanceInches;

  Thread armThread; Boolean isOn = true; Boolean isMade = false;
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
    armMotor1.setInverted(true);
    //pcm.enableCompressorDigital();
    pcm.disableCompressor();
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    camera.setPipelineIndex(2);
    ultrasonicPin_1.set(true);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
      currentDistanceInches = (short) Math.round(Distance.distCalc(ultrasonic.getValue()));
      System.out.println("Object distance in inches: " + currentDistanceInches);
      result = camera.getLatestResult();
       if (currentDistanceInches > 20.5) {
        if (result.hasTargets()) {
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
          } if (yaw <= -6.667) {
            driveTrain.tankDrive(-0.20, 0.20);
          } else if (yaw > -6.667 && yaw <= 6.666) {
            driveTrain.tankDrive(0.20, 0.20);
          } else if (yaw > 6.666 && yaw <= 30.0) {
            driveTrain.tankDrive(0.20, -0.20);
          }
        } else {
          driveTrain.tankDrive(0, 0);
        }
      }
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (!isMade) {
      armThread = new Thread(() -> {
        while (isOn) {
          //Put Arm Code Here
          if (operatorJoystick.getRawButton(5)) {
            piston_1.set(Value.kForward);
          } else if (operatorJoystick.getRawButton(4)) {
            piston_1.set(Value.kReverse);
          } else {
            piston_1.set(Value.kOff);
          } 
          armMotor1.set(operatorJoystick.getY());
        }
      });
    }
    isOn = true;
    armThread.setDaemon(true);
    armThread.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    driveTrain.tankDrive(leftJoystick.getY(), rightJoystick.getY());

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    isOn = false;
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
}
