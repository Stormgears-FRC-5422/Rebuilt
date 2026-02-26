package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class FieldConstants {
    public static boolean isAndyMark = true;
    static double leftOffset = 0.0;
    static double rightOffset = 0;

    public static double inchesToMeters(double x){return x*0.0254;};

    // Hub positions
    public static final Pose2d hubPositionRed = new Pose2d(11.912, 4.024, Rotation2d.fromDegrees(0));
    public static final Pose2d hubPositionBlue = new Pose2d(4.628, 4.024, Rotation2d.fromDegrees(0));

    // 4x4 ft from each field corner
    private static final double CORNER_OFFSET = Units.feetToMeters(4);
    public static final Pose2d leftBlue = new Pose2d(CORNER_OFFSET, RebuiltField.FIELD_WIDTH - CORNER_OFFSET, new Rotation2d());
    public static final Pose2d rightBlue = new Pose2d(CORNER_OFFSET, CORNER_OFFSET, new Rotation2d());
    public static final Pose2d leftRed = new Pose2d(RebuiltField.FIELD_LENGTH - CORNER_OFFSET, CORNER_OFFSET, new Rotation2d());
    public static final Pose2d rightRed = new Pose2d(RebuiltField.FIELD_LENGTH - CORNER_OFFSET, RebuiltField.FIELD_WIDTH - CORNER_OFFSET, new Rotation2d());





    /**
     * Method should NOT be used directly in commands
     * @param id the id of the tag
     */


    public static Pose2d getPoseTag(int id) {
    return switch (id) {
        case 1  -> new Pose2d(
                inchesToMeters(isAndyMark ? 467.08 : 467.64),
                inchesToMeters(isAndyMark ? 291.79 : 292.31),
                Rotation2d.fromDegrees(180.0));

        case 2  -> new Pose2d(
                inchesToMeters(isAndyMark ? 468.56 : 469.11),
                inchesToMeters(isAndyMark ? 182.08 : 182.60),
                Rotation2d.fromDegrees(90.0));

        case 3  -> new Pose2d(
                inchesToMeters(isAndyMark ? 444.80 : 445.35),
                inchesToMeters(isAndyMark ? 172.32 : 172.84),
                Rotation2d.fromDegrees(180.0));

        case 4  -> new Pose2d(
                inchesToMeters(isAndyMark ? 444.80 : 445.35),
                inchesToMeters(isAndyMark ? 158.32 : 158.84),
                Rotation2d.fromDegrees(180.0));

        case 5  -> new Pose2d(
                inchesToMeters(isAndyMark ? 468.56 : 469.11),
                inchesToMeters(isAndyMark ? 134.56 : 135.09),
                Rotation2d.fromDegrees(270.0));

        case 6  -> new Pose2d(
                inchesToMeters(isAndyMark ? 467.08 : 467.64),
                inchesToMeters(isAndyMark ? 24.85  : 25.37),
                Rotation2d.fromDegrees(180.0));

        case 7  -> new Pose2d(
                inchesToMeters(isAndyMark ? 470.03 : 470.59),
                inchesToMeters(isAndyMark ? 24.85  : 25.37),
                Rotation2d.fromDegrees(0.0));

        case 8  -> new Pose2d(
                inchesToMeters(isAndyMark ? 482.56 : 483.11),
                inchesToMeters(isAndyMark ? 134.56 : 135.09),
                Rotation2d.fromDegrees(270.0));

        case 9  -> new Pose2d(
                inchesToMeters(isAndyMark ? 492.33 : 492.88),
                inchesToMeters(isAndyMark ? 144.32 : 144.84),
                Rotation2d.fromDegrees(0.0));

        case 10 -> new Pose2d(
                inchesToMeters(isAndyMark ? 492.33 : 492.88),
                inchesToMeters(isAndyMark ? 158.32 : 158.84),
                Rotation2d.fromDegrees(0.0));

        case 11 -> new Pose2d(
                inchesToMeters(isAndyMark ? 482.56 : 483.11),
                inchesToMeters(isAndyMark ? 182.08 : 182.60),
                Rotation2d.fromDegrees(90.0));

        case 12 -> new Pose2d(
                inchesToMeters(isAndyMark ? 470.03 : 470.59),
                inchesToMeters(isAndyMark ? 291.79 : 292.31),
                Rotation2d.fromDegrees(0.0));

        case 13 -> new Pose2d(
                inchesToMeters(isAndyMark ? 649.58 : 650.92),
                inchesToMeters(isAndyMark ? 291.02 : 291.47),
                Rotation2d.fromDegrees(180.0));

        case 14 -> new Pose2d(
                inchesToMeters(isAndyMark ? 649.58 : 650.92),
                inchesToMeters(isAndyMark ? 274.02 : 274.47),
                Rotation2d.fromDegrees(180.0));

        case 15 -> new Pose2d(
                inchesToMeters(isAndyMark ? 649.57 : 650.90),
                inchesToMeters(isAndyMark ? 169.78 : 170.22),
                Rotation2d.fromDegrees(180.0));

        case 16 -> new Pose2d(
                inchesToMeters(isAndyMark ? 649.57 : 650.90),
                inchesToMeters(isAndyMark ? 152.78 : 153.22),
                Rotation2d.fromDegrees(180.0));

        case 17 -> new Pose2d(
                inchesToMeters(isAndyMark ? 183.03 : 183.59),
                inchesToMeters(isAndyMark ? 24.85  : 25.37),
                Rotation2d.fromDegrees(0.0));

        case 18 -> new Pose2d(
                inchesToMeters(isAndyMark ? 181.56 : 182.11),
                inchesToMeters(isAndyMark ? 134.56 : 135.09),
                Rotation2d.fromDegrees(270.0));

        case 19 -> new Pose2d(
                inchesToMeters(isAndyMark ? 205.32 : 205.87),
                inchesToMeters(isAndyMark ? 144.32 : 144.84),
                Rotation2d.fromDegrees(0.0));

        case 20 -> new Pose2d(
                inchesToMeters(isAndyMark ? 205.32 : 205.87),
                inchesToMeters(isAndyMark ? 158.32 : 158.84),
                Rotation2d.fromDegrees(0.0));

        case 21 -> new Pose2d(
                inchesToMeters(isAndyMark ? 181.56 : 182.11),
                inchesToMeters(isAndyMark ? 182.08 : 182.60),
                Rotation2d.fromDegrees(90.0));

        case 22 -> new Pose2d(
                inchesToMeters(isAndyMark ? 183.03 : 183.59),
                inchesToMeters(isAndyMark ? 291.79 : 292.31),
                Rotation2d.fromDegrees(0.0));

        case 23 -> new Pose2d(
                inchesToMeters(isAndyMark ? 180.08 : 180.64),
                inchesToMeters(isAndyMark ? 291.79 : 292.31),
                Rotation2d.fromDegrees(180.0));

        case 24 -> new Pose2d(
                inchesToMeters(isAndyMark ? 167.56 : 168.11),
                inchesToMeters(isAndyMark ? 182.08 : 182.60),
                Rotation2d.fromDegrees(90.0));

        case 25 -> new Pose2d(
                inchesToMeters(isAndyMark ? 157.79 : 158.34),
                inchesToMeters(isAndyMark ? 172.32 : 172.84),
                Rotation2d.fromDegrees(180.0));

        case 26 -> new Pose2d(
                inchesToMeters(isAndyMark ? 157.79 : 158.34),
                inchesToMeters(isAndyMark ? 158.32 : 158.84),
                Rotation2d.fromDegrees(180.0));

        case 27 -> new Pose2d(
                inchesToMeters(isAndyMark ? 167.56 : 168.11),
                inchesToMeters(isAndyMark ? 134.56 : 135.09),
                Rotation2d.fromDegrees(270.0));

        case 28 -> new Pose2d(
                inchesToMeters(isAndyMark ? 180.08 : 180.64),
                inchesToMeters(isAndyMark ? 24.85  : 25.37),
                Rotation2d.fromDegrees(180.0));

        case 29 -> new Pose2d(
                inchesToMeters(isAndyMark ? 0.54 : 0.30),
                inchesToMeters(isAndyMark ? 25.62 : 26.22),
                Rotation2d.fromDegrees(0.0));

        case 30 -> new Pose2d(
                inchesToMeters(isAndyMark ? 0.54 : 0.30),
                inchesToMeters(isAndyMark ? 42.62 : 43.22),
                Rotation2d.fromDegrees(0.0));

        case 31 -> new Pose2d(
                inchesToMeters(isAndyMark ? 0.55 : 0.32),
                inchesToMeters(isAndyMark ? 146.86 : 147.47),
                Rotation2d.fromDegrees(0.0));

        case 32 -> new Pose2d(
                inchesToMeters(isAndyMark ? 0.55 : 0.32),
                inchesToMeters(isAndyMark ? 163.86 : 164.47),
                Rotation2d.fromDegrees(0.0));

        default -> null;
    };


    }

    public static Pose2d hubPositionRED() {
        return hubPositionRed;
    }

    public static Pose2d hubPositionBLUE() {
        return hubPositionBlue;
    }

    public static Pose2d leftBlue() {
        return leftBlue;
    }

    public static Pose2d rightBlue() {
        return rightBlue;
    }

    public static Pose2d leftRed() {
        return leftRed;
    }

    public static Pose2d rightRed() {
        return rightRed;
    }








}
