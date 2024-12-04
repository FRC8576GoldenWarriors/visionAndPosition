// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AprilTagStats;
import frc.robot.subsystems.Drivetrain;

public class StandardAdjustRobotPos extends Command {
   private Drivetrain drivetrain;
   private AprilTagStats apriltag;


  public StandardAdjustRobotPos(Drivetrain drive, AprilTagStats apriltag) {
    drivetrain = drive;
    this.apriltag = apriltag;
    addRequirements(drivetrain);
    addRequirements(apriltag);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean targetIsFound = apriltag.hasTarget();
    apriltag.setTagView(targetIsFound);
    if (targetIsFound) {
        apriltag.updateData();
        //If possible, change to using PID for turn and forward
        double turn = (Constants.VisionConstants.distanceConstants.visionAngleDegrees - apriltag.getYaw()) * Constants.VisionConstants.VisionPIDConstants.kPVisionTurning;
        double forward = (Constants.VisionConstants.distanceConstants.goalMeterDistance - apriltag.getDistance()) * Constants.VisionConstants.VisionPIDConstants.kPVisionMoving;
        
        double sideSpeed = -RobotContainer.driverController.getLeftX() * Math.abs(RobotContainer.driverController.getLeftX()) * 1.8;
        sideSpeed = Math.abs(sideSpeed) > 0.15 ? sideSpeed : 0;
        drivetrain.swerveDrive(forward, sideSpeed, turn, true, new Translation2d(), false);;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.swerveDrive(
      0, 
      0, 
      0, 
      true, 
      new Translation2d(), 
      true
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

