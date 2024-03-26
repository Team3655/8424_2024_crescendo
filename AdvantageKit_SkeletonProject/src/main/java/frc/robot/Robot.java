// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.text.html.parser.DTD;

import org.littletonrobotics.junction.LoggedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final String spot3B = "spot3B";
  private static final String spot2B = "spot2B";
  private static final String spot1B = "spot1B";
  private static final String spot3R = "spot3R";
  private static final String spot2R = "spot2R";
  private static final String spot1R = "spot1R";
  private static final String shoot = "shoot";
  private static final String amp_auto = "amp_auto";
  private static final String blue_amp = "blue_amp";
  private static final String Red_amp = "Red_amp";
  private static final String speaker = "Speaker";
  private static final String Rank_point = "Rank_point";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static Timer timer = new Timer();

  private static CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private static CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private static CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private static CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private static CANSparkMax shootMotor = new CANSparkMax(5, MotorType.kBrushless);
  private static CANSparkMax shootMotorFollower = new CANSparkMax(6, MotorType.kBrushless);
  private static CANSparkMax pullMotor = new CANSparkMax(7, MotorType.kBrushless);
  private static CANSparkMax rotateMotor = new CANSparkMax(8, MotorType.kBrushless);

  private static DigitalInput ringSensor = new DigitalInput(0);

  private static RelativeEncoder rightEncoder1 = rightMotor1.getEncoder();
  private static RelativeEncoder leftEncoder1 = leftMotor1.getEncoder();
  private static RelativeEncoder rotateEncoder = rotateMotor.getEncoder();
  private static double circumference = 2 * Math.PI * Units.inchesToMeters(2.0);
  private static double gearRatio = 8.95;
  private static double rotateGearRatio = 100;

  private SparkPIDController pid = rotateMotor.getPIDController();

  private static GenericHID leftJoystick = new GenericHID(0);
  private static GenericHID rightJoystick = new GenericHID(1);
  private static GenericHID buttonJoystick = new GenericHID(2);
  // leftMotor1 and rightMotor1 are leaders
  private DifferentialDrive diffDrive = new DifferentialDrive(leftMotor1, rightMotor1);

  private final AHRS gyro = new AHRS(edu.wpi.first.wpilibj.SerialPort.Port.kMXP, SerialDataType.kProcessedData,
      (byte) 50);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(20.5));

  private DifferentialDrivePoseEstimator poseEstimator;

  // * This function is run when the robot is first started up and should be used
  // for any
  // * initialization code.
  // */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("spot3B", spot3B);
    m_chooser.addOption("spot2B", spot2B);
    m_chooser.addOption("spot1B", spot1B);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_chooser.addOption("spot3R", spot3R);
    m_chooser.addOption("spot1R", spot1R);
    m_chooser.addOption("spot2R", spot2R);
    m_chooser.addOption("shoot", shoot);
    m_chooser.addOption("amp_auto", amp_auto);
    m_chooser.addOption("blue_amp", blue_amp);
    m_chooser.addOption("Red_amp", Red_amp);
    m_chooser.addOption("speaker", speaker);
    m_chooser.addOption("Ranl_point", Rank_point);

    // Motor 2 will allways follow motor 1
    // dont use motor 2 in code

    leftMotor1.restoreFactoryDefaults();
    leftMotor2.restoreFactoryDefaults();
    rightMotor1.restoreFactoryDefaults();
    rightMotor2.restoreFactoryDefaults();

    shootMotor.restoreFactoryDefaults();
    shootMotorFollower.restoreFactoryDefaults();
    pullMotor.restoreFactoryDefaults();
    rotateMotor.restoreFactoryDefaults();

    shootMotor.setIdleMode(IdleMode.kCoast);

    leftMotor1.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kBrake);
    pullMotor.setIdleMode(IdleMode.kBrake);
    rotateMotor.setIdleMode(IdleMode.kBrake);

    leftMotor1.setInverted(true);
    rightMotor1.setInverted(false);
    shootMotorFollower.setInverted(true);

    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    rightEncoder1.setPositionConversionFactor(circumference / gearRatio);
    leftEncoder1.setPositionConversionFactor(circumference / gearRatio);
    rotateEncoder.setPositionConversionFactor(
        360 / rotateGearRatio); // Convert position of intake to degrees

    rotateEncoder.setPosition(0);

    pid.setP(.007);

    leftMotor1.burnFlash();
    leftMotor2.burnFlash();
    rightMotor1.burnFlash();
    rightMotor2.burnFlash();
    shootMotor.burnFlash();
    shootMotorFollower.burnFlash();
    rotateMotor.burnFlash();
    pullMotor.burnFlash();

    CameraServer.startAutomaticCapture(); // .setResolution(1280, 720);

    poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics,
        new Rotation2d(),
        leftEncoder1.getPosition(),
        rightEncoder1.getPosition(),
        new Pose2d());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("ringSensor", ringSensor.get());
    DifferentialDriveWheelPositions wheelPositions = new DifferentialDriveWheelPositions(
        leftEncoder1.getPosition(),
        rightEncoder1.getPosition());
    poseEstimator.update(gyro.getRotation2d(), wheelPositions);
    Pose2d robotPose = poseEstimator.getEstimatedPosition();
    SmartDashboard.putNumberArray("RobotPose", new double[] { robotPose.getTranslation().getX(),
        robotPose.getTranslation().getY(), robotPose.getRotation().getRadians() });
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    timer.reset();
    timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case Rank_point:
        if (timer.get() < .1) {
          pullMotor.set(.5);
          shootMotor.set(1);
          shootMotorFollower.set(1);
        } else if (timer.get() < 1) {
          shootMotor.set(1);
          shootMotorFollower.set(1);
        } else if (timer.get() < 2)
          ;

      case speaker:
        if (timer.get() < .1) { // 0.1 seconds
          pullMotor.set(.5);
          // sacrifice toothpick
        } else if (timer.get() < .3) { // 0.2 seconds
          pullMotor.set(0);
          // stop intake
        } else if (timer.get() < 2) { // 1.7 sec

          shootMotorFollower.set(1);
          shootMotor.set(1);
          // warm up shooter
        } else if (timer.get() < 2.5) { // 0.5 sec

          pullMotor.set(-.5);

          // release the note

        } else if (timer.get() < 6) { // 1.5 sec

          // rightShooter.set(0);
          // leftShooter.set(0);
          pullMotor.set(0);
          rotateMotor.getPIDController().setReference(-235, ControlType.kPosition);

          // stop intake and shooter
        } else if (timer.get() < 8) {
          pullMotor.set(.3);
          diffDrive.arcadeDrive(-0.5, 0);

          // drive backwards and intake note

        } else if (timer.get() < 9) { // 1

          diffDrive.arcadeDrive(0, 0);
          rotateMotor.getPIDController().setReference(0, ControlType.kPosition);

          // stop driving and close lifter
          // Postive is out

        } else if (timer.get() < 10.5) { // 1.5 sec

          shootMotor.set(1);
          shootMotorFollower.set(1);
          diffDrive.arcadeDrive(0.5, 0);

          // warm up shooter AND drive forward

        } else if (timer.get() < 13) { // 1 sec

          pullMotor.set(-.5);

          // Feed the note

        } else if (timer.get() < 15) { // 1.5 sec

          shootMotor.set(0); // right shooter motor OFF
          shootMotorFollower.set(0); // left shooter motor OFF
          pullMotor.set(0); // intake OFF
          diffDrive.arcadeDrive(-0.6, 0); // drive backwards -0.6 power

          // Shooter & intake off, drive backwards to cross line

          // } else if (timer.get() < 9.5) { // 0.5 seconds
          // diffDrive.arcadeDrive(-0.7, 0.3);
          // try to turn and drive backwards???
          // } else if (timer.get() < 12) { // 2.5 seconds
          // diffDrive.arcadeDrive(-0.7, 0);
          // turn slightly to avoid
          // } else if (timer.get() < 12.5) { // 0.5 seconds
          // drive back at middle line
          // diffDrive.arcadeDrive(-0.7, -0.3);
          // turn back pointing at note
          // } else if (timer.get() < 14) diffDrive.arcadeDrive(7, 0);
          // else {

          // diffDrive.arcadeDrive(0, 0); // drive OFF

          // stop and hang out! :)
        }
        break;

      case Red_amp:
        if (timer.get() < .1) {
          // brake toothpicks
          pullMotor.set(.5);
          // readys the motor
        } else if (timer.get() < .2) {
          pullMotor.set(0);
          shootMotor.set(.23);
          shootMotorFollower.set(.23);
        } else if (timer.get() < 1.9) {
          diffDrive.arcadeDrive(.5, 0.5);
        } else if (timer.get() < 2.4) {
          diffDrive.arcadeDrive(.5, 0);
        } else if (timer.get() < 3.4) {
          diffDrive.arcadeDrive(0, 0);
        } else if (timer.get() < 5.4) {
          pullMotor.set(-.5);
        } else if (timer.get() < 6.4) {
          rotateMotor.getPIDController().setReference(-235, ControlType.kPosition);
        } else if (timer.get() < 6.9) {
          diffDrive.arcadeDrive(-.5, 0);

        } else if (timer.get() < 8.7) {
          diffDrive.arcadeDrive(-.5, .5);
          pullMotor.set(.5);
        } else if (timer.get() < 9.2) {
          pullMotor.set(.5);

        } else if (timer.get() < 11) {
          pullMotor.set(0);
          diffDrive.arcadeDrive(.5, -0.5);
        } else if (timer.get() < 12.2) {
          rotateMotor.getPIDController().setReference(0, ControlType.kPosition);
          diffDrive.arcadeDrive(.5, 0);
          shootMotor.set(.23);
          shootMotorFollower.set(.23);
        } else if (timer.get() > 13.3) {
          pullMotor.set(-.5);

        } else {

        }
        break;
      case blue_amp:
        if (timer.get() < .1) {
          // brake toothpicks
          pullMotor.set(.5);
          // readys the motor
        } else if (timer.get() < .2) {
          pullMotor.set(0);
          shootMotor.set(.23);
          shootMotorFollower.set(.23);
        } else if (timer.get() < 1.9) {
          diffDrive.arcadeDrive(.5, -.5);
        } else if (timer.get() < 2.4) {
          diffDrive.arcadeDrive(.5, 0);
        } else if (timer.get() < 3.4) {
          diffDrive.arcadeDrive(0, 0);
        } else if (timer.get() < 5.4) {
          pullMotor.set(-.5);
        } else if (timer.get() < 6.4) {
          rotateMotor.getPIDController().setReference(-235, ControlType.kPosition);
        } else if (timer.get() < 6.9) {
          diffDrive.arcadeDrive(-.5, 0);

        } else if (timer.get() < 8.7) {
          diffDrive.arcadeDrive(-.5, -.5);
          pullMotor.set(.5);
        } else if (timer.get() < 9.2) {
          pullMotor.set(.5);

        } else if (timer.get() < 11) {
          pullMotor.set(0);
          diffDrive.arcadeDrive(.5, .5);
        } else if (timer.get() < 12.2) {
          rotateMotor.getPIDController().setReference(0, ControlType.kPosition);
          diffDrive.arcadeDrive(.5, 0);
          shootMotor.set(.23);
          shootMotorFollower.set(.23);
        } else if (timer.get() > 13.3) {
          pullMotor.set(-.5);

        } else {

        }

        break;

      // case amp_auto:
      // Clear toothpics from intake
      // if (timer.get() < .1) {
      // pullMotor.set(.5);
      // move torawds the amp
      // } else if (timer.get() < .2) {
      // pullMotor.set(0);
      // } else if (timer.get() < 1.2) {
      // diffDrive.arcadeDrive(.8, 0);

      // } else if (timer.get() < 1.45) {
      // diffDrive.arcadeDrive(0, .5);

      // } else if (timer.get() < 1.5) {
      // diffDrive.arcadeDrive(-.5, 0);

      // } else if (timer.get() < 3.5) {
      // shootMotor.set(.25);
      // shootMotorFollower.set(.25);
      // } else if (timer.get() < 5) {
      // pullMotor.set(-.5);
      // } else if (timer.get() < 5.2) {
      // shootMotor.set(0);
      // shootMotorFollower.set(0);
      // pullMotor.set(0);
      // diffDrive.arcadeDrive(.5, 0);

      // } else if (timer.get() < 5.45) {
      // diffDrive.arcadeDrive(0, -.5);
      // } else if (timer.get() < 8.45) {
      // diffDrive.arcadeDrive(.8, 0);
      // rotateMotor
      // .getPIDController()
      // .setReference(-210, ControlType.kPosition); // TODO: Check position of intake
      // pullMotor.set(.5);

      // } else if (timer.get() < 9) {
      // diffDrive.arcadeDrive(0, 0);
      // } else if (!ringSensor.get()) {
      // pullMotor.set(0);

      // } else if (timer.get() < 11) {
      // diffDrive.arcadeDrive(-.8, 0);
      // } else if (timer.get() < 11.25) {
      // diffDrive.arcadeDrive(0, .5);
      // } else if (timer.get() < 11.5) {
      // diffDrive.arcadeDrive(.5, 0);
      // shootMotor.set(1);
      // shootMotorFollower.set(1);
      // } else if (timer.get() > 14) {
      // pullMotor.set(-.5);
      // }

      // break;

      case shoot:
        // Clear toothpics from intake
        if (timer.get() < .1) {
          pullMotor.set(.5);
          // stop intake
        } else if (timer.get() < 6) {
          pullMotor.set(0);
        }
        // spin up motors for 3 seconds
        // } else if(timer.get() < 9){
        // shootMotor.set(1);
        // shootMotorFollower.set(1);
        // release note and fire!
        // } else if (timer.get() > 10) {
        // pullMotor.set(-.5);
        // }
      default:
        break;

      case spot2B:
        // clear toothpick from intake
        if (timer.get() < .1) {
          pullMotor.set(.5);
        }
        // stop intake and warm up moters
        else if (timer.get() < 2) {
          pullMotor.set(0);
          shootMotor.set(1);
          shootMotorFollower.set(1);
        }
        // feeed shooters
        else if (timer.get() < 3.6) {
          pullMotor.set(-.5);
          // Move back
        } else if (timer.get() < 8) {
          diffDrive.arcadeDrive(-.5, 0);
          // stop moving
        } else if (timer.get() < 9.5) {
          diffDrive.arcadeDrive(0, 0);
        }
        break;

      case spot3B:
        // sacrifice toothpick
        if (timer.get() < .1) {
          pullMotor.set(.5);
          // stop intake and speed up shooter
        } else if (timer.get() < 3) {
          pullMotor.set(0);
          shootMotor.set(1);
          shootMotorFollower.set(1);
        }
        // release note and fire!
        else if (timer.get() < 4) {
          pullMotor.set(-.5);
          // drive back
        } else if (timer.get() < 8) {
          diffDrive.arcadeDrive(-.3, 0);
          // rotate to the _____
        } else if (timer.get() < 8.25) {
          diffDrive.arcadeDrive(0, .5);
          // Drive back faster
        } else if (timer.get() < 12) {
          diffDrive.arcadeDrive(-.5, 0);
          // stop moving
        } else {
          diffDrive.arcadeDrive(0, 0);
        }
        break;

      case spot1B:
        // sacrifice toothpick
        if (timer.get() < .1) {
          pullMotor.set(.5);
          // sacrificing complete & warm up speakers
        } else if (timer.get() < 1.5) {
          pullMotor.set(0);
          shootMotor.set(1);
          shootMotorFollower.set(1);
          // release note and fire!
        } else if (timer.get() < 5) {
          pullMotor.set(-.5);
          // drive back
        } else if (timer.get() < 9) {
          diffDrive.arcadeDrive(-.5, 0);
          // rotate to the _____
        } else if (timer.get() < 9.25) {
          diffDrive.arcadeDrive(0, -.5);
          // drive back
        } else if (timer.get() < 13) {
          diffDrive.arcadeDrive(-.5, 0);
          // stop moving
        } else {
          diffDrive.arcadeDrive(0, 0);
        }
        break;

      case spot3R:
        // sacrifice toothpick
        if (timer.get() < .1) {
          pullMotor.set(.5);
          // stop intake and speed up shooter
        } else if (timer.get() < 3) {
          pullMotor.set(0);
          shootMotor.set(1);
          shootMotorFollower.set(1);
          // release note and fire!
        } else if (timer.get() < 3.5) {
          pullMotor.set(-.5);
          // drive back
        } else if (timer.get() < 7) {
          diffDrive.arcadeDrive(-.3, 0);
          // rotate to the _______
        } else if (timer.get() < 7.25) {
          diffDrive.arcadeDrive(0, -.5);
          // drive back
        } else if (timer.get() < 11) {
          diffDrive.arcadeDrive(-.5, 0);
          // stop
        } else {
          diffDrive.arcadeDrive(0, 0);
        }
        break;

      case spot2R:
        // Sacrifice the toothpick
        if (timer.get() < .1) {
          pullMotor.set(.5);
          // stop intake and speed up shooters
        } else if (timer.get() < 1) {
          pullMotor.set(0);
          shootMotor.set(1);
          shootMotorFollower.set(1);
          // wait!!!
        } else if (timer.get() < 4) {
          pullMotor.set(0);
          // release note and fire!
        } else if (timer.get() < 4.5) {
          pullMotor.set(-.5);
          // drive back
        } else if (timer.get() < 9) {
          diffDrive.arcadeDrive(-.5, 0);
          // stop driving
        } else if (timer.get() < 9.5) {
          diffDrive.arcadeDrive(0, 0);
        }
        break;

      case spot1R:
        if (timer.get() < .1) {
          pullMotor.set(.5);

        } else if (timer.get() < 1.25) {
          pullMotor.set(0);
          shootMotor.set(1);
          shootMotorFollower.set(1);

        } else if (timer.get() < 3) {
          pullMotor.set(-.5);

        } else if (timer.get() < 7) {
          diffDrive.arcadeDrive(-.3, 0);

        } else if (timer.get() < 7.25) {
          diffDrive.arcadeDrive(0, .5);

        } else if (timer.get() < 11) {
          diffDrive.arcadeDrive(-.5, 0);

        } else {
          diffDrive.arcadeDrive(0, 0);
        }
        break;
    }
  }

  // spot3LPickup:
  // if(timer.get() < 2.5){
  // shootMotor.set(1);
  // shootMotorFollower.set(1);
  // } else if (timer.get() < 3) {
  // pullMotor.set(-.5);
  // }

  // else if (timer.get() < 4){
  // shootMotor.set(.0);
  // shootMotorFollower.set(0);
  // pullMotor.set(0);

  // } else if (timer.get() < 4.8); {
  // diffDrive.arcadeDrive(.8, 0);

  // } else if (timer.get() < 4.9); {
  // diffDrive.arcadeDrive(0, .8);

  // } else if (timer.get() <5.5); {
  // diffDrive.arcadeDrive(.8, 0);

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("Intake Position", rotateMotor.getEncoder().getPosition());

    if (buttonJoystick.getRawButton(5)) {
      rotateMotor.getPIDController().setReference(-235, ControlType.kPosition);
    }
    // rotateMotor.set(buttonJoystick.getRawAxis(1) * 0.6);

    if (buttonJoystick.getRawButton(10)) {
      rotateMotor.getPIDController().setReference(0, ControlType.kPosition);
    }

    if (buttonJoystick.getRawButton(3)) {
      pullMotor.set(-.6);
    } else if (buttonJoystick.getRawButton(2)) {
      pullMotor.set(.6);
    } else {
      pullMotor.set(0);
    }

    if (buttonJoystick.getRawButton(1)) {
      shootMotor.set(1);
      shootMotorFollower.set(1);
    } else if (buttonJoystick.getRawButton(4)) {
      shootMotor.set(.23);
      shootMotorFollower.set(.23);
    } else if (buttonJoystick.getRawButton(6)) {
      shootMotor.set(-.5);
      shootMotorFollower.set(-.5);
    } else {
      shootMotor.set(0);

      shootMotorFollower.set(0);
    }
    diffDrive.arcadeDrive(leftJoystick.getRawAxis(1) * 1, rightJoystick.getRawAxis(0) * .8);

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
