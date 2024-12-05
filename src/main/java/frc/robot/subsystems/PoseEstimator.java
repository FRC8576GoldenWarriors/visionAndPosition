package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimator extends SubsystemBase {
    private final Drivetrain drivetrain;
    private Pose2d visionPose;
    private static SwerveDrivePoseEstimator poseEstimator;
    public PoseEstimator(){
    drivetrain = Drivetrain.getInstance();
    // Maxswerve Version from MAXSwerve.java in core
    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.DRIVE_KINEMATICS,
            drivetrain.getHeadingRotation2d(),
            drivetrain.getModulePositions(),
            drivetrain.getPose2d(),
            Matrix.mat(Nat.N3(), Nat.N1())
            .fill(
                // if these numbers are less than one, multiplying will do bad things
                Constants.SwerveConstants.PoseConfig.kVisionStdDevX,
                Constants.SwerveConstants.PoseConfig.kVisionStdDevY,
                Constants.SwerveConstants.PoseConfig.kVisionStdDevTheta
            ),

            Matrix.mat(Nat.N3(), Nat.N1())
            .fill(
                // if these numbers are less than one, multiplying will do bad things
                Constants.SwerveConstants.PoseConfig.kPositionStdDevX,
                Constants.SwerveConstants.PoseConfig.kPositionStdDevY,
                Constants.SwerveConstants.PoseConfig.kPositionStdDevTheta
            ));
  }

  @Override
  public void periodic() {
    poseEstimator.update(drivetrain.getHeadingRotation2d(),drivetrain.getModulePositions()); // Updates using wheel encoder data only
    // Updates using the vision estimate
    visionPose = AprilTagStats.getInstance().getRobotPose().toPose2d();
    
      double currentTimestamp =
          AprilTagStats.getInstance().getTimeStamp(AprilTagStats.getInstance().getLatency());
      if (Constants.VisionConstants.distanceConstants.useableIDs.contains(AprilTagStats.getInstance().getID())) {
        poseEstimator.addVisionMeasurement(visionPose, currentTimestamp);
      }
}
public static SwerveDrivePoseEstimator getPoseEstimator(){
    return poseEstimator;
}
}