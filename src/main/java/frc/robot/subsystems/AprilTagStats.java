package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants.cameraRotationConstants;
import frc.robot.Constants.VisionConstants.cameraTranslationConstants;

public class AprilTagStats extends SubsystemBase {
    //Creating new object for the arducam
    private PhotonCamera m_arduCam;
    
    //Object representation of the field
    private AprilTagFieldLayout m_layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    
    //Positional object representation of the camera on the robot
    private final Transform3d m_robotToCam = new Transform3d(new Translation3d(cameraTranslationConstants.tX, cameraTranslationConstants.tY, cameraTranslationConstants.tZ), new Rotation3d(cameraRotationConstants.rRoll, cameraRotationConstants.rPitch, cameraRotationConstants.rYaw));
    
    //Photon Pose Estimator object
    private PhotonPoseEstimator m_photonPoseEstimator;

    //Shuffleboard tab named vision, and 4 different widgets for yaw, pitch, tag id, and distance to a tag
    private ShuffleboardTab m_tab;
    private GenericEntry m_yawEntry, m_pitchEntry, m_idEntry, m_distanceEntry, m_tagStatusEntry;

    //Variables to hold all of the widget values
    private double m_yaw, m_pitch, m_distance;
    private int m_id;
    private boolean m_tagStatus;

    //Generic starting position of the robot
    // private final Pose3d m_startPose3d = new Pose3d(12.5, 5.5, 0, new Rotation3d());
    public static final AprilTagStats apriltagstats = new AprilTagStats();
    private StructPublisher<Pose3d> m_publisher;
    private PhotonTrackedTarget target;
    private Pose3d relativePose;
    public AprilTagStats(){
        m_publisher = NetworkTableInstance.getDefault().getStructTopic(Constants.VisionConstants.nameConstants.publishName, Pose3d.struct).publish();
        m_tab = Shuffleboard.getTab(Constants.VisionConstants.nameConstants.tabName);
        m_arduCam = new PhotonCamera(Constants.VisionConstants.nameConstants.cameraName);
        m_yawEntry = m_tab.add("yaw", m_yaw).getEntry();
        m_pitchEntry = m_tab.add("pitch", m_pitch).getEntry();
        m_idEntry = m_tab.add("id", m_id).getEntry();
        m_distanceEntry = m_tab.add("distance", m_distance).getEntry();
        m_tagStatusEntry = m_tab.add("Tag In View", m_tagStatus).getEntry();
        
        m_photonPoseEstimator = new PhotonPoseEstimator(m_layout, PoseStrategy.CLOSEST_TO_LAST_POSE, m_arduCam, m_robotToCam);
    }
    public AprilTagStats(String cameraName, String publishName, String tabName) {
        m_publisher = NetworkTableInstance.getDefault().getStructTopic(publishName, Pose3d.struct).publish();
        m_tab = Shuffleboard.getTab(tabName);
        m_arduCam = new PhotonCamera(cameraName);
        m_yawEntry = m_tab.add("yaw", m_yaw).getEntry();
        m_pitchEntry = m_tab.add("pitch", m_pitch).getEntry();
        m_idEntry = m_tab.add("id", m_id).getEntry();
        m_distanceEntry = m_tab.add("distance", m_distance).getEntry();
        m_tagStatusEntry = m_tab.add("Tag In View", m_tagStatus).getEntry();
        
        m_photonPoseEstimator = new PhotonPoseEstimator(m_layout, PoseStrategy.CLOSEST_TO_LAST_POSE, m_arduCam, m_robotToCam);
    }

    private PhotonTrackedTarget getTarget() {
        return m_arduCam.getLatestResult().getBestTarget();
    }

    public static AprilTagStats getInstance(){
        return apriltagstats;
    }
    public boolean pingCam() {
        return m_arduCam.isConnected();
    }

    public void updateData() {
        //pushes yaw, pitch, id, and distance between the robot and tag to ShuffleBoard (Meant for testing if values are being passed to variables)
        m_yawEntry.setDouble(getYaw());
        m_pitchEntry.setDouble(getPitch());
        m_idEntry.setInteger(getID());
        m_distanceEntry.setDouble(getDistance());
    }

    public Pose3d getRobotPose() {
        //intakes the space between a camera and its target, the target itself, the camera's position and rotation on the robot, and the field. Outputs the robot relative to the field.
        target = getTarget();
        Pose3d estimatedPose = new Pose3d();
        int id = getID();
        if (id != -1 && target != null && m_layout.getTagPose(id).isPresent()) {
            estimatedPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), m_layout.getTagPose(id).get(), m_robotToCam);
        }
        return estimatedPose;
    }
    public PathPlannerPath robotPath(){
        List<Translation2d> waypoints = null;
        Pose2d tagPose = m_layout.getTagPose(m_id).get().toPose2d();
        if(m_id==7){
        waypoints = PathPlannerPath.bezierFromPoses(
        getRobotPose().toPose2d(),
        new Pose2d(tagPose.getX()+Constants.VisionConstants.distanceConstants.goalMeterDistance,tagPose.getY(),tagPose.getRotation())
);
        }
        else if(m_id==4){
        waypoints = PathPlannerPath.bezierFromPoses(
        getRobotPose().toPose2d(),
        new Pose2d(tagPose.getX()-Constants.VisionConstants.distanceConstants.goalMeterDistance,tagPose.getY(),tagPose.getRotation())
);
        }

PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
// PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

// Create the path using the waypoints created above
if(waypoints!=null){
PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        new GoalEndState(0.0, Rotation2d.fromDegrees(0))
        ,true 
);
return path;
}
return null;
    }
    public void updatePose() {
        //Updates the robots pose on network tables
        m_publisher.set(getRobotPose());
    }

    public double getDistance() {
        //Gets the distance between the robot and the wall of the apriltag its viewing
        double tagPitch = getPitch();
        relativePose = getRobotPose();
        int id = getID();
        if (id != -1 && relativePose != null) {
            double hypotenuse = PhotonUtils.getDistanceToPose(relativePose.toPose2d(), m_layout.getTagPose(getID()).get().toPose2d());
            return hypotenuse * Math.cos(Math.toRadians(tagPitch));
        }
        return 0;
    }

    public double getYaw() {
        target = getTarget();
        if (target != null) {
            return target.getYaw();
        }
        return 0;
    }

    public double getPitch() {
        target = getTarget();
        if (target != null) {
            return target.getPitch();
        }
        return 0;
    }

    public int getID() {
        target = getTarget();
        if (target != null) {
            return target.getFiducialId();
        }
        return -1;
    }

    public boolean hasTarget() {
        if (m_arduCam.hasTargets() && pingCam()) {
            return true;
        }
        return false;
    }

    public void setTagView(boolean tagFound) {
        if (tagFound) {
            m_tagStatusEntry.setBoolean(true);
        }
        else {
            m_tagStatusEntry.setBoolean(false);
        }
    }
    public double getTimeStamp(double latency){
        return Timer.getFPGATimestamp()-(latency/1000d);
    }
    public double getLatency(){
        return m_arduCam.getLatestResult().getLatencyMillis();
    }

    public void updateView() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateView'");
    }
}