package frc.robot.subsystems.drive.config;

import edu.wpi.first.math.geometry.Translation2d;

public class ModuleConfig {
    private boolean isDirty = false;
    public int encoderID = 0;
    public int driveID = 0;
    public int steerID = 0;
    public boolean driveInverted = false;
    public boolean steerInverted = false;
    public double driveRatio = 1;
    double steerRatio = 1;
    public double wheelDiameter = 0;
    public boolean driveCoast = true;
    public boolean steerCoast = true;
    public double encoderOffset = 0;
    public boolean invertEncoder = false;
    public Translation2d offset = new Translation2d();
    public String name = "module";

    public void setDirty() {
        isDirty = true;
    }

    public void clearDirty() {
        isDirty = false;
    }

    public boolean isDirty() {
        return isDirty;
    }

    public static ModuleConfig from(ModuleConfig value) {
        return new ModuleConfig() {{
            name = value.name;
            driveID = value.driveID;
            steerID = value.steerID;
            encoderID = value.encoderID;
            driveInverted = value.driveInverted;
            steerInverted = value.steerInverted;
            driveRatio = value.driveRatio;
            steerRatio = value.steerRatio;
            wheelDiameter = value.wheelDiameter;
            driveCoast = value.driveCoast;
            steerCoast = value.steerCoast;
            encoderOffset = value.encoderOffset;
            invertEncoder = value.invertEncoder;
            offset = new Translation2d(value.offset.getX(), value.offset.getY());
        }};
    }

    public ModuleConfig withName(String newName) {
        name = newName;
        return this;
    }
    public ModuleConfig withDriveID(int newID) {
        driveID = newID;
        return this;
    }

    public ModuleConfig withSteerID(int newID) {
        steerID = newID;
        return this;
    }

    public ModuleConfig withEncoderID(int newID) {
        encoderID = newID;
        return this;
    }

    public ModuleConfig withDriveInverted(boolean newInverted) {
        driveInverted = newInverted;
        return this;
    }

    public ModuleConfig withSteerInverted(boolean newInverted) {
        steerInverted = newInverted;
        return this;
    }

    public ModuleConfig withDriveRatio(double newDriveRatio) {
        driveRatio = newDriveRatio;
        return this;
    }

    public ModuleConfig withSteerRatio(double newSteerRatio) {
        steerRatio = newSteerRatio;
        return this;
    }

    public ModuleConfig withDriveCoast(boolean newCoast) {
        driveCoast = newCoast;
        return this;
    }

    public ModuleConfig withSteerCoast(boolean newCoast) {
        steerCoast = newCoast;
        return this;
    }

    public ModuleConfig withWheelDiameter(double newDiameter) {
        wheelDiameter = newDiameter;
        return this;
    }

    public ModuleConfig withEncoderOffset(double newEncoderOffset) {
        encoderOffset = newEncoderOffset;
        return this;
    }

    public ModuleConfig withInvertEncoder(boolean newInvertEncoder) {
        invertEncoder = newInvertEncoder;
        return this;
    }

    public ModuleConfig withOffset(Translation2d newOffset) {
        offset = new Translation2d(newOffset.getX(), newOffset.getY());
        return this;
    }

    @Override
    public String toString() {
        String ss = "Module Config (" + name + ")\n";
        ss += "          driveID: " + driveID + "\n";
        ss += "          steerID: " + steerID + "\n";
        ss += "        encoderID: " + encoderID + "\n";
        ss += "    driveInverted: " + driveInverted + "\n";
        ss += "    steerInverted: " + steerInverted + "\n";
        ss += "    wheelDiameter: " + wheelDiameter + "\n";
        ss += "       driveCoast: " + driveCoast + "\n";
        ss += "       steerCoast: " + steerCoast + "\n";
        ss += "    encoderOffset: " + encoderOffset + "\n";
        ss += "    invertEncoder: " + invertEncoder + "\n";
        ss += "           offset: " + offset + "\n";
        return ss;
    }
}
