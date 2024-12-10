// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AdjustRobotPos;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.FFShooterAngle;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.OverrideIntakeDown;
import frc.robot.commands.OverrideIntakeUp;
import frc.robot.commands.SetShooterAmp;
import frc.robot.commands.SetShooterAngle;
import frc.robot.commands.ShooterDown;
import frc.robot.commands.ShooterUp;
import frc.robot.commands.StandardAdjustRobotPos;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.AprilTagStats;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.PhasingLEDPattern;
import frc.robot.subsystems.PhyscialLEDStrip;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.RainbowLEDPattern;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRoller;



public class RobotContainer {

  public static final Drivetrain drivetrain = Drivetrain.getInstance();


  public static final Intake m_Intake = new Intake();
  public static final IntakeRoller m_IntakeRoller = new IntakeRoller();

  public static final Shooter m_Shooter = new Shooter();
  public static final ShooterRoller m_ShooterRoller = new ShooterRoller();
  public static final Climber m_Climber = new Climber();
  public static final AprilTagStats m_AprilTag = new AprilTagStats(Constants.VisionConstants.nameConstants.cameraName, Constants.VisionConstants.nameConstants.publishName,Constants.VisionConstants.nameConstants.tabName);
  public static final PoseEstimator m_poseEstimator = new PoseEstimator();
  public final LEDStrip ledStrip;
 // public static final LED m_led = new LED(Constants.LEDConstants.LED_PORT1, Constants.LEDConstants.LedLength1);
 

  public static final CommandXboxController driverController = new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  public static final CommandXboxController operatorController = new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);

  private final JoystickButton resetHeading_Start = new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);

  public RobotContainer() {

    ledStrip = new PhyscialLEDStrip(9, 58); //58

    configureBindings();

    drivetrain.setDefaultCommand(new SwerveDrive());
  }


  private void configureBindings() {
    
    //Driver controller
    resetHeading_Start.onTrue(
      new InstantCommand(drivetrain::zeroHeading, drivetrain));

    ledStrip.setDefaultCommand(new RunCommand(() -> {
       
      if (!m_IntakeRoller.getDigitalInput().get()){//note is loaded
        if(m_ShooterRoller.isRevved()){ //shooter is activated -> cyan
          ledStrip.usePattern(new PhasingLEDPattern(new Color8Bit(57, 190, 165), 1.0));
        }
        else if(m_ShooterRoller.isAmping()){// shooter in amp state -> purple
          ledStrip.usePattern(new PhasingLEDPattern(new Color8Bit(255, 0, 200), 1.0));
        }
        else { //shooter is not running -> green
          ledStrip.usePattern(new PhasingLEDPattern(new Color8Bit(44, 255,10), 0.5));
        }
      }
      else if (m_Climber.getRainbowBoolean()){ //if climbers are extending -> rainbow
          ledStrip.usePattern(new RainbowLEDPattern(5, 7));
        }
       
      else if(RobotState.isDisabled()){ //when robot is disabled -> yellow
          ledStrip.usePattern(new PhasingLEDPattern(new Color8Bit(255, 255, 0), 0.5));
      }
      else{
        // default -> red
        ledStrip.usePattern(new PhasingLEDPattern(new Color8Bit(255, 0, 0
        ), 0.5));
      }
      



    }, ledStrip)); 
      
    //Operator Controller

     // Intake
    //operatorController.a().whileTrue(new IntakeUp(m_Intake));
    // operatorController.a().whileTrue(new IntakeDown(m_Intake));//new SequentialCommandGroup(new IntakeIn(m_Intake), new IntakeUp(m_Intake), new Shoot(m_Shooter)));
    // operatorController.x().whileTrue(new IntakeUp(m_Intake));


    //default
    operatorController.a().onTrue(new SequentialCommandGroup(
      new IntakeDown(m_Intake),
      new IntakeIn(m_IntakeRoller).until(() -> !m_IntakeRoller.getDigitalInput().get()), 
      new IntakeIn(m_IntakeRoller).withTimeout(0.8), 
      new ParallelCommandGroup(
        new InstantCommand(() -> m_ShooterRoller.setSpeed(0.525)),
        new InstantCommand(() -> m_ShooterRoller.setRevved(true)), 
        new IntakeUp(m_Intake), 
        new SetShooterAngle(m_Shooter, 0.045)))); //0.015


     //High angle, same speed - Source to mid launch - assembly line pt1
    operatorController.x().onTrue(new SequentialCommandGroup(
      new IntakeDown(m_Intake),
      new IntakeIn(m_IntakeRoller).until(() -> !m_IntakeRoller.getDigitalInput().get()), 
      new IntakeIn(m_IntakeRoller).withTimeout(0.8), 
      new ParallelCommandGroup(
        new InstantCommand(() -> m_ShooterRoller.setSpeed(0.525)),
        new InstantCommand(() -> m_ShooterRoller.setRevved(true)), 
        new IntakeUp(m_Intake), 
        new SetShooterAngle(m_Shooter, 0.058))));

    //assembly line pt2
    operatorController.y().onTrue(new SequentialCommandGroup(
      new IntakeDown(m_Intake),
      new IntakeIn(m_IntakeRoller).until(() -> !m_IntakeRoller.getDigitalInput().get()), 
      new IntakeIn(m_IntakeRoller).withTimeout(0.8), 
      new ParallelCommandGroup(
        new InstantCommand(() -> m_ShooterRoller.setSpeed(0.4)),
        new InstantCommand(() -> m_ShooterRoller.setRevved(true)), 
        new IntakeUp(m_Intake), 
        new SetShooterAngle(m_Shooter, 0.015))));



    //operatorController.a().whileTrue(new IntakeIn(m_IntakeRoller));
    //operatorController.x().whileTrue(new IntakeIn(m_IntakeRoller));

    operatorController.y().and(operatorController.leftBumper()).whileTrue(new ShooterUp(m_Shooter));
    operatorController.b().and(operatorController.leftBumper()).whileTrue(new ShooterDown(m_Shooter));

    operatorController.b().onTrue(new InstantCommand(() -> m_ShooterRoller.setSpeed(0)));


    driverController.povDown().whileTrue(new OverrideIntakeUp(m_Intake));
    driverController.povUp().whileTrue(new OverrideIntakeDown(m_Intake));

    //Shooter
  
    //operatorController.y().onTrue(new Shoot(m_ShooterRoller)); //b button ends shoot command, defined in shoot command
    //Shintake
    // operatorController.povDown().whileTrue(new Shintake(m_Shooter));
    
    //Climber
    //Window button is button #7. Retracts the climber.
    // operatorController.back().whileTrue(new Climb(m_Climber));
    driverController.b().whileTrue(new Climb(m_Climber));
    // //Three line button is button #8. Extends the climber.
    driverController.a().whileTrue((new ClimbDown(m_Climber)));

    driverController.rightBumper().whileTrue(new IntakeOut(m_IntakeRoller));

    driverController.button(7).onTrue(new InstantCommand(() -> m_Shooter.zeroEncoder()));

    //Vision
    //Basic Code:
    // driverController.x()
    // .and(()->m_AprilTag.getID()==4||m_AprilTag.getID()==7) 
    // .onTrue(new AdjustRobotPos(drivetrain, m_AprilTag));
    operatorController.leftBumper()
    .and(()->Constants.VisionConstants.distanceConstants.useableIDs.contains(m_AprilTag.getID()))
    .onTrue(new AdjustRobotPos(drivetrain, m_AprilTag));
    // operatorController.leftBumper()
    //  .and(()->Constants.VisionConstants.distanceConstants.useableIDs.contains(m_AprilTag.getID()))
    // .onTrue(new SequentialCommandGroup(new ParallelCommandGroup(new AdjustRobotPos(drivetrain, m_AprilTag),new SetShooterAngle(m_Shooter,Constants.ShooterConstants.kShooterAutoAngle)),new Shoot(m_ShooterRoller)));
    //Figure out after there's a failsafe to not finding apriltags driverController.x().onTrue(new SequentialCommandGroup(new ParallelCommandGroup(new AdjustRobotPos(drivetrain, m_AprilTag),new SetShooterAngle(m_Shooter,Constants.ShooterConstants.kShooterAutoAngle)),new Shoot(m_ShooterRoller)));
    // operatorController.start().whileTrue(new ClimbDown(m_Climber));
    //bindings for running non-path planner vision movement
    operatorController.rightBumper().and(operatorController.a()).and(()->Constants.VisionConstants.distanceConstants.useableIDs.contains(m_AprilTag.getID())).onTrue(new StandardAdjustRobotPos(drivetrain, m_AprilTag));

    //driverController.y().onTrue(getAutonomousCommand())
    
    //Arm
    //Commented bindings match the documented bindings
    // operatorController.leftBumper().and(operatorController.a()).whileTrue( new  OverrideIntakeDown(m_Intake));
    // operatorController.leftBumper().and(operatorController.x()).whileTrue( new OverrideIntakeUp(m_Intake));
    operatorController.a().and(operatorController.leftBumper()).onTrue(new IntakeUp(m_Intake));
    operatorController.x().and(operatorController.leftBumper()).onTrue(new IntakeDown(m_Intake));

    //test code for shooter pivot

    operatorController.rightBumper().whileTrue(new IntakeIn(m_IntakeRoller));
    
    

    operatorController.povLeft().onTrue(new SetShooterAngle(m_Shooter, 0.015)); //0.015
    operatorController.povUp().onTrue(new SetShooterAngle(m_Shooter, 0.058));
    operatorController.povRight().onTrue(new SetShooterAmp(m_Shooter, 0.097, m_ShooterRoller));

    

    //operatorController.povLeft().onTrue(new FFShooterAngle(m_Shooter, 0.015));
    operatorController.povDown().onTrue(new FFShooterAngle(m_Shooter, 0.055));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}