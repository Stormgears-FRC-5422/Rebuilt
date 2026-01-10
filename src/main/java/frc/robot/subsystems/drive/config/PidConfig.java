package frc.robot.subsystems.drive.config;
/*
 * This class was shamelessly stolen from the Slot0Configs class from CTRE.
 * Simplified for the outreach robot
 */

public class PidConfig {

    private boolean isDirty = false;
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kS = 0;
    public double kV = 0;
    public double kA = 0;
    public double kMin = 0;
    public double kMax = 0;

    public void setDirty() {
        isDirty = true;
    }

    public void clearDirty() {
        isDirty = false;
    }

    public boolean isDirty() {
        return isDirty;
    }

    public static PidConfig from(PidConfig value) {
        return new PidConfig() {
            {
                kP = value.kP;
                kI = value.kI;
                kD = value.kD;
                kS = value.kS;
                kV = value.kV;
                kA = value.kA;
                kMin = value.kMin;
                kMax = value.kMax;
            }
        };
    }

    public PidConfig withKP(double newKP) {
        kP = newKP;
        return this;
    }

    public PidConfig withKI(double newKI) {
        kI = newKI;
        return this;
    }

    public PidConfig withKD(double newKD) {
        kD = newKD;
        return this;
    }

    public PidConfig withKS(double newKS) {
        kS = newKS;
        return this;
    }

    public PidConfig withKV(double newKV) {
        kV = newKV;
        return this;
    }

    public PidConfig withKA(double newKA) {
        kA = newKA;
        return this;
    }

    public PidConfig withKMinMax(double newKMin, double newKMax) {
        kMin = newKMin;
        kMax = newKMax;
        return this;
    }

    @Override
    public String toString() {
        String ss = "PID Config:\n";
        ss += "       kP: " + kP + "\n";
        ss += "       kI: " + kI + "\n";
        ss += "       kD: " + kD + "\n";
        ss += "       kS: " + kS + "\n";
        ss += "       kV: " + kV + "\n";
        ss += "       kA: " + kA + "\n";
        ss += "  kMinOut: " + kMin + "\n";
        ss += "  kMaxOut: " + kMax + "\n";
        return ss;
    }
}
