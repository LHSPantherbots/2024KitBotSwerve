package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightNoteDect extends SubsystemBase {
    NetworkTable table;

    private boolean validTargets = false;
    private double horizontalOffset;
    private double verticalOffset;
    // private double targetArea;
    private double cameraHeight = 0.508;
    private double cameraAngle = 0.0;

    public LimeLightNoteDect() {
        table = NetworkTableInstance.getDefault().getTable("limelight-wilson");
    }

    @Override
    public void periodic() {
        validTargets = isTargetValid();
        horizontalOffset = getHorizontalOffset();
        verticalOffset = getVerticalOffset();
        // targetArea = getTargetArea();
        SmartDashboard.putBoolean("Wilson Valid Target", validTargets);
        if (validTargets) {
            SmartDashboard.putString("Target Translation", getTargetTranslation().toString());
        }
    }

    public Translation2d getTargetTranslation() {
        var x = (-cameraHeight)/Math.tan(Math.toRadians(cameraAngle+verticalOffset));
        var y = x*Math.sin(Math.toRadians(horizontalOffset));
        return new Translation2d(x, y);
    }

    public boolean isTargetValid() {
        return (table.getEntry("tv").getDouble(0) == 1);
    }

    public double getHorizontalOffset() {
        return table.getEntry("tx").getDouble(0);
    }

    public double getVerticalOffset() {
        return table.getEntry("ty").getDouble(0);
    }

    public double getTargetArea() {
        return table.getEntry("ta").getDouble(0);
    }

    public double getTargetLatency() {
        return table.getEntry("tl").getDouble(0);
    }

    public double getCaptureLatency() {
        return table.getEntry("cl").getDouble(0);
    }
}
