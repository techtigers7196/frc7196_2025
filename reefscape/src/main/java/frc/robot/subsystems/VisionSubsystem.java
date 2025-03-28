package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem {

    private  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-left");
    private  NetworkTableEntry tx = table.getEntry("tx");
    private  NetworkTableEntry ty = table.getEntry("ty");
    private  NetworkTableEntry ta = table.getEntry("ta");
    private NetworkTableEntry tv = table.getEntry("tv");
    private NetworkTableEntry tPose = table.getEntry("botpose_targetspace");


    private  NetworkTable tableRight = NetworkTableInstance.getDefault().getTable("limelight-right");
    private  NetworkTableEntry txRight = tableRight.getEntry("tx");
    private  NetworkTableEntry tyRight = tableRight.getEntry("ty");
    private  NetworkTableEntry taRight = tableRight.getEntry("ta");
    private NetworkTableEntry tvRight = table.getEntry("tv");

    public double[] getXYA()
    {
        double x = tx.getDouble(0.0);
        double y  = ty.getDouble(0.0);
        double a = ta.getDouble(0.0);
        return new double[]{x, y, a};
    }

    public double[] getXYARight()
    {
        double x = txRight.getDouble(0.0);
        double y  = tyRight.getDouble(0.0);
        double a = taRight.getDouble(0.0);
        return new double[]{x, y, a};
    }

    public double [] getCameraPose(){
        return tPose.getDoubleArray(new double[6]);
    }


    public boolean hasTarget()
    {
        return tv.getInteger(0) == 1 || tvRight.getInteger(0) == 1;
    }
    

    // public void processMegatagVision(SwerveSubsystem swerveDrive){
    //     double [] botpose = table.getEntry("botpose").getDoubleArray(new double[6]);
    //     double latency = table.getEntry("tl").getDouble(0) / 1000.0;
    //     boolean hasTarget = table.getEntry("tv").getInteger(0) ==1;

    //     if (hasTarget && botpose.length >=6){
    //         Pose3d visionPose = new Pose3d(
    //             botpose[0], botpose [1], botpose[2],
    //             new Rotation3d(
    //                 Math.toRadians(botpose[3]),
    //                 Math.toRadians(botpose[4]),
    //                 Math.toRadians(botpose[5])

    //             )
    //         );

    //         double timestamp = Timer.getFPGATimestamp() - latency;

    //         swerveDrive.getSwerveDrive().addVisionMeasurement(visionPose.toPose2d(), timestamp);
    //     }
    // }

}

