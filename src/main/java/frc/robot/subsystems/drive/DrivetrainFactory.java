package frc.robot.subsystems.drive;

import frc.robot.subsystems.drive.AkitDrive.AkitDrive;

public class DrivetrainFactory {
    protected static DrivetrainBase instance;

    public static DrivetrainBase getInstance(String driveType, String driveSubtype) throws IllegalDriveTypeException {
        if (instance == null) {
            System.out.println("Initializing " + driveType);
            if (driveSubtype != null && !driveSubtype.isEmpty()) {
                System.out.println("Ignoring drive subtype " + driveSubtype);
            }
            switch (driveType.toLowerCase()) {
                case "diagnosticswerve" -> instance = new DiagnosticSwerve();
                case "basicswerve" -> instance = new BasicSwerve();
                case "akitdrive" -> instance = new AkitDrive();
                default -> throw new IllegalDriveTypeException("Illegal Drive Type: " + driveType);
            }
        }
        return instance;
    }
}
