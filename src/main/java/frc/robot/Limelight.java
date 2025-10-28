package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    private NetworkTable table;
    private Mode mode;

    public enum Mode {
        CAMERA,
        SENSOR
    }

    public Limelight(Mode initialMode) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        setMode(initialMode);
    }

    public Mode getMode() {
        return mode;
    }

    public void setMode(Mode mode) {
        int camMode, ledMode;

        switch (mode) {
            case CAMERA:
                camMode = 1;
                ledMode = 1;
                break;

            case SENSOR:
                camMode = 0;
                ledMode = 3;
                break;

            default:
                return;
        }

        table.getEntry("camMode").setNumber(camMode);
        table.getEntry("ledMode").setNumber(ledMode);

        this.mode = mode;
    }

    public void toggleMode() {
        switch (mode) {
            case CAMERA:
                setMode(Mode.SENSOR);
                break;
            case SENSOR:
                setMode(Mode.CAMERA);
                break;
        }
    }

    /**
     * @return Horizontal offset from crosshair to target (-27 degrees to 27 degrees)
     */
    public double getX() {
        return table.getEntry("tx").getDouble(0.0);
    }

    /**
     * @return Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees)
     */
    public double getY() {
        return table.getEntry("ty").getDouble(0.0);
    }

    /**
     * @return Whether the limelight can see valid targets
     */
    public boolean targetVisible() {
        return (table.getEntry("tv").getDouble(0) == 1);
    }

}
