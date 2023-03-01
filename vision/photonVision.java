import java.lang.annotation.Target;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public class photonVision 
{
    String camera_name = "cameraZ";

    // Creates a photon camera w/ same name as camera
    PhotonCamera camera = new PhotonCamera(camera_name);


    public PhotonPipelineResult getLResult()
    {
        // Query the latest result from PhotonVision (huh? -z)
        // Info about currently detected targets
        // returns a container (query? -z) of information w/ timestamp
        PhotonPipelineResult result = camera.getLatestResult();
        return result;

        // should "result" be an instance variable?
        // should camera be a parameter that gets passed in?
    }

    public boolean hasTargets()
    {
        // This checks if the latest result(!!! -z) has any targets.
        boolean hasTargets = result.hasTargets();
        return hasTargets;

        // should result be a parameter that gets passed in?
    }

    /* 
        What is a PhotonTrackedTarget???
        A tracked target contains information about each target from a pipeline result. 
        Information includes:
            yaw, 
            pitch, 
            area, 
            and robot relative pose.

        // results are pipelined in?
    */ 
    
    public List<PhotonTrackedTarget> getTargets() {
        // Get a list of currently tracked targets.
        List<PhotonTrackedTarget> targets = result.getTargets();
        return targets;
    }

    public PhotonTrackedTarget bestTarget()
    {
        // Get the current best target.
        PhotonTrackedTarget target = result.getBestTarget();
        return target;
    }

    // what is this?
    public double getYaw(Target target)
    {
        return target.getYaw();
    }

    // what is this?
    public double getPitch(Target target)
    {
        return target.getPitch();
    }

    public double getArea(Target target)
    {
        return target.getArea();
    }

    public double getSkew(Target target)
    {
        return target.getSkew();
    }

    // look at docs for Transform2d 
    // https://docs.wpilib.org/en/latest/docs/software/advanced-controls/geometry/transformations.html#transform2d-and-twist2d
    public Transform2d getPose(Target target)
    {
        Transform2d pose = target.getCameraToTarget();
        return pose;
    }
    
    public List<TargetCorner> getCorners(Target target)
    {
        List<TargetCorner> corners = target.getCorners();
    }


    // Getting April Tag Data:
    public int getID(Target target)
    {
        // The ID of the detected fiducial marker.
        int targetID = target.getFiducialId();
        return targetID;
    }

    public double poseAmbig(Target target)
    {
        double poseAmbiguity = target.getPoseAmbiguity();
        return poseAmbiguity;
    }
    
    public Transform3d bestCamera2Target(Target target)
    {
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        return bestCameraToTarget;
    }
    
    public Transform3d altCamera2Target(Target target)
    {
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
        return alternateCameraToTarget;
    }
    
    // Say click, Take a pic
    // https://www.youtube.com/watch?v=gktOri8vP7E&t=9s
    public void inputSnap()
    {
        // Capture pre-process camera stream image
        camera.takeInputSnapshot();
    }

    public void outputSnap()
    {
        // Capture post-process camera stream image
        camera.takeOutputSnapshot();
    }





    
    

}


