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

    private  NetworkTable tableRight = NetworkTableInstance.getDefault().getTable("limelight-right");
    private  NetworkTableEntry txRight = table.getEntry("tx");
    private  NetworkTableEntry tyRight = table.getEntry("ty");
    private  NetworkTableEntry taRight = table.getEntry("ta");

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

}

