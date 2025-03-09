package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem {

    private  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private  NetworkTableEntry tx = table.getEntry("tx");
    private  NetworkTableEntry ty = table.getEntry("ty");
    private  NetworkTableEntry ta = table.getEntry("ta");
    private NetworkTableEntry tPose = table.getEntry("camerapose_targetspace");

    public void VisionPeriodic(){
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double[] pose = getCameraPose();

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumberArray("LimelightPoseEst", pose);
    }

    public double[] getXYA()
    {
        double x = tx.getDouble(0.0);
        double y  = ty.getDouble(0.0);
        double a = ta.getDouble(0.0);
        return new double[]{x, y, a};
    }

    public double[] getCameraPose()
    {
        return tPose.getDoubleArray(new double[6]);
    }

}

