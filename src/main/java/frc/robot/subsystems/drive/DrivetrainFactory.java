package frc.robot.subsystems.drive;

import frc.robot.subsystems.drive.basicSwerve.BasicSwerve;
import frc.robot.subsystems.drive.basicSwerve.DiagnosticSwerve;
import frc.robot.subsystems.drive.ctrDrive.CTRSwerve;
import frc.robot.subsystems.drive.tunerConstants.RebuiltVoltage;

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
                case "ctrswerve" -> instance = new CTRSwerve(getTunerConstants(driveSubtype));
                default -> throw new IllegalDriveTypeException("Illegal Drive Type: " + driveType);
            }
        }
        return instance;
    }

    // Right now there is only one kind of subtype. This could be more complicated if that changes.
    // We could just make a different get function for each driveType, but for now these should all work
    // with ctrdrive or akdrive types
    private static Class<?> getTunerConstants(String subtype) throws IllegalDriveTypeException {
        System.out.println("Collected constants for subtype: " + subtype);
        return switch (subtype.toLowerCase()) {
            case "ctrrebuilt" -> RebuiltVoltage.class;
            default -> throw new IllegalDriveTypeException("Illegal Drive subtype: " + subtype);
        };
    }

}
