// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.sql.Driver;

//import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final Victor victor1 = new Victor(1);
  private final Victor victor3 = new Victor(3);
  private final Victor victor2 = new Victor(2);
  private final Victor victor4 = new Victor(4);
  private final XboxController xbox = new XboxController(0);
  private final XboxController ddr = new XboxController(1);

  // add possible victor inversions here
  
  // ^^^^^^^^

  private final MecanumDrive mechDrive = new MecanumDrive(victor1, victor2, victor3, victor4); 
  //private final CANSparkMax sparkMax1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax sparkBase = new CANSparkMax(26, MotorType.kBrushless);
  private final CANSparkMax sparkForearm = new CANSparkMax(17, MotorType.kBrushless);
  private final CANSparkMax sparkHands = new CANSparkMax(57, MotorType.kBrushless);
  private RelativeEncoder encoderBase = sparkBase.getEncoder();  
  private RelativeEncoder encoderForearm = sparkForearm.getEncoder();    
  //  private final CANSparkMax sparkMax3 = new CANSparkMax(53, MotorType.kBrushless);
  //The weird device ids was to fix a strange problem where all the ids would reset on boot, and would get confused     
  //Spark 1: Base / First arm motor, Spark 2: 2nd Arm Motor / Middle Spark 3: Hand 
  private RobotContainer m_robotContainer;
  public double forwardVelocity = 0; 
  public double turnVelocity = 0;
  public double sideVelocity = 0;
  public int forwardCont = 0;
  public int sideCont = 0;
  public int turnCont = 0;
  public double maxSpeed = 0;
  boolean moving = false;
  boolean arm = false;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    sparkBase.setIdleMode(IdleMode.kBrake);
    sparkForearm.setIdleMode(IdleMode.kBrake);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    SmartDashboard.setDefaultNumber("speed", 0.5);
    SmartDashboard.setDefaultNumber("Forearm Position", 0);
    SmartDashboard.setDefaultNumber("Base Position", 0);
    victor1.setInverted(true);
    victor3.setInverted(true);   
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    maxSpeed = SmartDashboard.getNumber("speed", 0);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //sparkMax2.set(.1);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    motorSet();
    SmartDashboard.putNumber("Base Position", encoderBase.getPosition());
    SmartDashboard.putNumber("Forearm Position", encoderForearm.getPosition());    
    if(xbox.getRightTriggerAxis() > .5 && !arm)
      {
        mechDrive.driveCartesian(-xbox.getLeftX() * maxSpeed, xbox.getLeftY() * maxSpeed, 0);
        if (xbox.getRightX() != 0)
        {
          turnBetter(xbox.getRightX());
          
        }
      }
    if(xbox.getRightTriggerAxis() > .5 && xbox.getLeftTriggerAxis() > .5)
    {
      mechDrive.driveCartesian(0, 0, 0);
      sparkBase.set(0);
      sparkForearm.set(0);
      DriverStation.reportError("Dual Input (L-TriggerAxis & R-TriggerAxis)", false);
    }
    if(xbox.getLeftTriggerAxis() > .5 && xbox.getRightTriggerAxis() < .5)
      {
        arm = true;
        sparkBase.set(xbox.getLeftY() * .1);
        sparkForearm.set(xbox.getRightY() * .1);
      }
    if(xbox.getLeftTriggerAxis() < .5)
      {
        if(arm == true)
        {
        sparkBase.set(0);
        sparkForearm.set(0);
        arm = false;
        }
      }
    if(xbox.getLeftBumper())
      {
        moving = true;
        sparkHands.set(-.5);
      
      }
    if(xbox.getRightBumper())
      {
        
        moving = true;
        sparkHands.set(.5);
      }
    if(!(xbox.getLeftBumper() || xbox.getRightBumper()))
    {
      sparkHands.set(0);
      moving = false;

    }
    if(xbox.getRightTriggerAxis() > .5 && ((forwardCont + sideCont + turnCont) > 0) )
    {
      DriverStation.reportWarning("Overriding DDR Pad (R-Axis > 0.5)", false);
      
    }
    if(xbox.getRightTriggerAxis() < .5)
    {
      mechDrive.driveCartesian(forwardCont * maxSpeed, sideCont * maxSpeed, 0);
      if (turnCont != 0){
        turnBetter(turnCont);
      }
    }
  }

  public void motorSet()
  {
    forwardCont = boolToInt(ddr.getRawButton(1)) - boolToInt(ddr.getRawButton(4));
    sideCont = boolToInt(ddr.getRawButton(2)) - boolToInt(ddr.getRawButton(3));
    turnCont = boolToInt(ddr.getRawButton(8)) - boolToInt(ddr.getRawButton(7));
  }

  public void turnBetter(double input)
  {
    victor1.set(-input * .5);
    victor2.set(input * .5);
    victor3.set(-input * .5);
    victor4.set(input * .5);
  }

  public int boolToInt(boolean b) {
    return Boolean.compare(b, false);
  }
 
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

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