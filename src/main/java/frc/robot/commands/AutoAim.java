package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagStats;
import frc.robot.subsystems.Shooter;

public class AutoAim extends Command{
    private Shooter shooter;
    private AprilTagStats apriltag;
    private double angle;
    public AutoAim(Shooter shooter, AprilTagStats apriltag){
        this.shooter = shooter;
        this.apriltag = apriltag;
        addRequirements(shooter);
        addRequirements(apriltag);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = Math.tan(211/apriltag.getDistance()*100.0);
    new SetShooterAngle(shooter,angle);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new SetShooterAngle(shooter, 0);
    //climber.setRainbowBoolean(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getAbsoluteDistance()==angle;
  }

}
