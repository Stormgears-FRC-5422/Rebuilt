package frc.robot.simulation.collision;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.game.RebuiltField;

import java.util.ArrayList;
import java.util.List;

/**
 * Defines the 2026 REBUILT field boundaries and obstacles for collision detection.
 *
 * <p>Strategy: Obstacles are defined once for the BLUE alliance, then automatically
 * rotated to create RED alliance obstacles using RebuiltField's transformation
 * functions. This ensures field symmetry and reduces code duplication.
 *
 * <p>Field coordinate system:
 * <ul>
 *   <li>Origin at blue alliance corner (X=0, Y=0)</li>
 *   <li>X-axis along field length (blue → red)</li>
 *   <li>Y-axis along field width (+Y is LEFT when facing red)</li>
 *   <li>All dimensions in meters</li>
 * </ul>
 */
public class FieldCollisionMap {
    private final CollisionGeometry.FieldBoundary boundary;
    private final List<CollisionGeometry.ConvexPolygon> obstacles;

    public FieldCollisionMap() {
        boundary = new CollisionGeometry.FieldBoundary(
            0,
            RebuiltField.FIELD_LENGTH,
            0,
            RebuiltField.FIELD_WIDTH
        );

        obstacles = new ArrayList<>();
        addObstacles();
    }

    private void addObstacles() {
        final double INCH_TO_M = 0.0254;

        // Hub: 47" square, 158.6" from alliance wall (to near edge)
        double hubSize = 47.0 * INCH_TO_M;
        double blueHubX = (158.6 * INCH_TO_M) + (hubSize / 2.0);
        CollisionGeometry.ConvexPolygon blueHub = createRectangle(
            blueHubX, RebuiltField.FIELD_WIDTH / 2.0, hubSize, hubSize);
        obstacles.add(blueHub);

        // Trench supports: 47" deep, 15.31" thick, 50.34" from side walls (to near edge)
        double trenchDepth = 47.0 * INCH_TO_M;
        double trenchThickness = 15.31 * INCH_TO_M;  // 65.65" - 50.34" clearance
        double trenchOffset = (50.34 * INCH_TO_M) + (trenchThickness / 2.0);
        CollisionGeometry.ConvexPolygon blueTrenchLeft = createRectangle(
            blueHubX, trenchOffset, trenchDepth, trenchThickness);
        CollisionGeometry.ConvexPolygon blueTrenchRight = createRectangle(
            blueHubX, RebuiltField.FIELD_WIDTH - trenchOffset, trenchDepth, trenchThickness);
        obstacles.add(blueTrenchLeft);
        obstacles.add(blueTrenchRight);

        // Tower: 39" wide, 45" deep, positioned with two 72" driver stations to the left
        double towerWidth = 39.0 * INCH_TO_M;
        double towerDepth = 45.0 * INCH_TO_M;
        double towerY = RebuiltField.FIELD_WIDTH - (2.0 * 72.0 * INCH_TO_M) - (6.0 * INCH_TO_M) - (towerWidth / 2.0);
        CollisionGeometry.ConvexPolygon blueTower = createRectangle(
            towerDepth / 2.0, towerY, towerDepth, towerWidth);
        obstacles.add(blueTower);

        // Red obstacles (180-degree rotation)
        obstacles.add(rotateObstacle(blueHub));
        obstacles.add(rotateObstacle(blueTrenchLeft));
        obstacles.add(rotateObstacle(blueTrenchRight));
        obstacles.add(rotateObstacle(blueTower));
    }

    private CollisionGeometry.ConvexPolygon rotateObstacle(CollisionGeometry.ConvexPolygon blueObstacle) {
        Translation2d[] blueVertices = blueObstacle.getVertices();
        Translation2d[] redVertices = new Translation2d[blueVertices.length];

        for (int i = 0; i < blueVertices.length; i++) {
            redVertices[i] = RebuiltField.rotateBlueToRed(blueVertices[i]);
        }

        return new CollisionGeometry.ConvexPolygon(redVertices);
    }

    /** Create axis-aligned rectangle obstacle (meters). */
    public CollisionGeometry.ConvexPolygon createRectangle(double centerX, double centerY,
                                                           double sizeX, double sizeY) {
        double halfX = sizeX / 2.0;
        double halfY = sizeY / 2.0;

        return new CollisionGeometry.ConvexPolygon(
            new Translation2d(centerX - halfX, centerY - halfY),
            new Translation2d(centerX + halfX, centerY - halfY),
            new Translation2d(centerX + halfX, centerY + halfY),
            new Translation2d(centerX - halfX, centerY + halfY)
        );
    }

    /** Create convex polygon obstacle from vertices (counter-clockwise). */
    public CollisionGeometry.ConvexPolygon createPolygon(Translation2d... vertices) {
        return new CollisionGeometry.ConvexPolygon(vertices);
    }

    public CollisionGeometry.FieldBoundary getBoundary() {
        return boundary;
    }

    public List<CollisionGeometry.ConvexPolygon> getObstacles() {
        return obstacles;
    }

    public void addObstacle(CollisionGeometry.ConvexPolygon obstacle) {
        obstacles.add(obstacle);
    }

    public void clearObstacles() {
        obstacles.clear();
    }
}
