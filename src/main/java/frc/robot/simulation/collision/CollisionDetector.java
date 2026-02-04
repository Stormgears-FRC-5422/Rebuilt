package frc.robot.simulation.collision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Collision detection system for robot simulation.
 * Uses two-phase detection: AABB fast rejection + SAT precise collision.
 */
public class CollisionDetector {
    private final FieldCollisionMap fieldMap;
    private final CollisionGeometry.RobotBounds robotBounds;

    /**
     * Create a collision detector with robot dimensions in inches.
     * @param robotLengthInches Robot length including bumpers (inches)
     * @param robotWidthInches Robot width including bumpers (inches)
     */
    public CollisionDetector(double robotLengthInches, double robotWidthInches) {
        this.fieldMap = new FieldCollisionMap();
        this.robotBounds = new CollisionGeometry.RobotBounds(
            Units.inchesToMeters(robotLengthInches),
            Units.inchesToMeters(robotWidthInches)
        );
    }

    /**
     * Check if a pose would cause a collision.
     * Uses two-phase detection for performance.
     * @param candidatePose The pose to check
     * @return CollisionResult indicating if collision occurred and type
     */
    public CollisionResult checkCollision(Pose2d candidatePose) {
        CollisionGeometry.RobotBounds transformedRobot = robotBounds.at(candidatePose);
        CollisionGeometry.AABB robotAABB = transformedRobot.getBoundingBox();

        // Check field boundaries (fast AABB containment)
        if (!fieldMap.getBoundary().contains(transformedRobot)) {
            return new CollisionResult(true, CollisionType.FIELD_BOUNDARY, null);
        }

        // Check obstacles with two-phase detection
        for (CollisionGeometry.ConvexPolygon obstacle : fieldMap.getObstacles()) {
            // PHASE 1: Fast AABB rejection test (O(1))
            if (!robotAABB.overlaps(obstacle.getBoundingBox())) {
                continue; // No AABB overlap, skip expensive SAT
            }

            // PHASE 2: Precise SAT collision (only if AABBs overlap)
            if (transformedRobot.intersects(obstacle)) {
                return new CollisionResult(true, CollisionType.OBSTACLE, obstacle);
            }
        }

        return CollisionResult.NO_COLLISION;
    }

    /**
     * Constrain a pose to be collision-free.
     * Implements sliding behavior along boundaries and obstacles.
     * @param desiredPose The desired pose
     * @param previousPose The previous valid pose
     * @return Constrained pose that doesn't collide
     */
    public Pose2d constrainPose(Pose2d desiredPose, Pose2d previousPose) {
        CollisionResult result = checkCollision(desiredPose);

        if (!result.hasCollision()) {
            return desiredPose;
        }

        if (result.type == CollisionType.FIELD_BOUNDARY) {
            return clampToFieldBoundary(desiredPose, previousPose);
        } else {
            return slideAlongObstacle(desiredPose, previousPose, result.obstacle);
        }
    }

    /**
     * Clamp robot position to field boundaries.
     * Allows sliding along edges by clamping X and Y independently.
     * @param desiredPose The desired pose
     * @param previousPose The previous valid pose
     * @return Clamped pose within field boundaries
     */
    private Pose2d clampToFieldBoundary(Pose2d desiredPose, Pose2d previousPose) {
        CollisionGeometry.FieldBoundary boundary = fieldMap.getBoundary();
        CollisionGeometry.RobotBounds robot = robotBounds.at(desiredPose);

        // Get robot corners to determine maximum safe translation
        Translation2d[] corners = robot.getCorners();

        // Find the extent of the robot in each direction
        double minX = Double.MAX_VALUE;
        double maxX = Double.MIN_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = Double.MIN_VALUE;

        for (Translation2d corner : corners) {
            minX = Math.min(minX, corner.getX());
            maxX = Math.max(maxX, corner.getX());
            minY = Math.min(minY, corner.getY());
            maxY = Math.max(maxY, corner.getY());
        }

        // Calculate offsets from robot center to extents
        double offsetMinX = desiredPose.getX() - minX;
        double offsetMaxX = maxX - desiredPose.getX();
        double offsetMinY = desiredPose.getY() - minY;
        double offsetMaxY = maxY - desiredPose.getY();

        // Clamp robot center position accounting for extents
        double clampedX = MathUtil.clamp(
            desiredPose.getX(),
            boundary.getMinX() + offsetMinX,
            boundary.getMaxX() - offsetMaxX
        );

        double clampedY = MathUtil.clamp(
            desiredPose.getY(),
            boundary.getMinY() + offsetMinY,
            boundary.getMaxY() - offsetMaxY
        );

        Pose2d clampedPose = new Pose2d(clampedX, clampedY, desiredPose.getRotation());

        // Verify clamped pose is valid
        if (checkCollision(clampedPose).hasCollision()) {
            // Clamping failed, return previous pose
            return previousPose;
        }

        return clampedPose;
    }

    /**
     * Slide along an obstacle edge when collision detected.
     * Projects movement vector onto obstacle edge tangent.
     * @param desiredPose The desired pose
     * @param previousPose The previous valid pose
     * @param obstacle The obstacle being collided with
     * @return Pose that slides along the obstacle
     */
    private Pose2d slideAlongObstacle(Pose2d desiredPose, Pose2d previousPose,
                                     CollisionGeometry.ConvexPolygon obstacle) {
        // Compute movement vector
        Translation2d movement = desiredPose.getTranslation().minus(previousPose.getTranslation());

        // If no movement, just return previous pose
        if (movement.getNorm() < 0.0001) {
            return previousPose;
        }

        // Find nearest obstacle edge
        Translation2d[] vertices = obstacle.getVertices();
        Translation2d nearestEdge = null;
        double minDistance = Double.MAX_VALUE;

        Translation2d robotCenter = desiredPose.getTranslation();

        for (int i = 0; i < vertices.length; i++) {
            Translation2d p1 = vertices[i];
            Translation2d p2 = vertices[(i + 1) % vertices.length];

            // Edge vector
            Translation2d edge = p2.minus(p1);

            // Vector from p1 to robot center
            Translation2d toRobot = robotCenter.minus(p1);

            // Project robot onto edge
            double t = toRobot.getX() * edge.getX() + toRobot.getY() * edge.getY();
            t /= (edge.getX() * edge.getX() + edge.getY() * edge.getY());
            t = MathUtil.clamp(t, 0, 1);

            // Closest point on edge
            Translation2d closest = p1.plus(edge.times(t));

            // Distance to edge
            double distance = closest.getDistance(robotCenter);

            if (distance < minDistance) {
                minDistance = distance;
                nearestEdge = edge;
            }
        }

        if (nearestEdge == null) {
            return previousPose;
        }

        // Normalize edge to get tangent
        double edgeLength = nearestEdge.getNorm();
        if (edgeLength < 0.0001) {
            return previousPose;
        }
        Translation2d tangent = nearestEdge.div(edgeLength);

        // Project movement onto tangent
        double projection = movement.getX() * tangent.getX() + movement.getY() * tangent.getY();
        Translation2d projectedMovement = tangent.times(projection);

        // Try sliding pose
        Pose2d slidPose = new Pose2d(
            previousPose.getTranslation().plus(projectedMovement),
            desiredPose.getRotation()
        );

        // Verify slide pose is valid
        if (checkCollision(slidPose).hasCollision()) {
            // Sliding still causes collision, stay at previous pose
            return previousPose;
        }

        return slidPose;
    }

    /**
     * Get field boundary as Pose2d array for visualization.
     * @return Array of poses representing field boundary corners
     */
    public Pose2d[] getFieldBoundaryPoses() {
        CollisionGeometry.FieldBoundary boundary = fieldMap.getBoundary();
        return new Pose2d[] {
            new Pose2d(boundary.getMinX(), boundary.getMinY(), new edu.wpi.first.math.geometry.Rotation2d()),
            new Pose2d(boundary.getMaxX(), boundary.getMinY(), new edu.wpi.first.math.geometry.Rotation2d()),
            new Pose2d(boundary.getMaxX(), boundary.getMaxY(), new edu.wpi.first.math.geometry.Rotation2d()),
            new Pose2d(boundary.getMinX(), boundary.getMaxY(), new edu.wpi.first.math.geometry.Rotation2d()),
            new Pose2d(boundary.getMinX(), boundary.getMinY(), new edu.wpi.first.math.geometry.Rotation2d()) // Close the loop
        };
    }

    /**
     * Get obstacle as Pose2d array for visualization.
     * @param index Index of obstacle
     * @return Array of poses representing obstacle vertices
     */
    public Pose2d[] getObstaclePoses(int index) {
        if (index < 0 || index >= fieldMap.getObstacles().size()) {
            return new Pose2d[0];
        }

        CollisionGeometry.ConvexPolygon obstacle = fieldMap.getObstacles().get(index);
        Translation2d[] vertices = obstacle.getVertices();

        Pose2d[] poses = new Pose2d[vertices.length + 1]; // +1 to close the loop
        for (int i = 0; i < vertices.length; i++) {
            poses[i] = new Pose2d(vertices[i], new edu.wpi.first.math.geometry.Rotation2d());
        }
        // Close the loop
        poses[vertices.length] = poses[0];

        return poses;
    }

    /**
     * Get robot bounds as Pose2d array for visualization.
     * @param robotPose Current robot pose
     * @return Array of poses representing robot footprint
     */
    public Pose2d[] getRobotBoundsPoses(Pose2d robotPose) {
        CollisionGeometry.RobotBounds robot = robotBounds.at(robotPose);
        Translation2d[] corners = robot.getCorners();

        Pose2d[] poses = new Pose2d[5]; // 4 corners + close the loop
        for (int i = 0; i < 4; i++) {
            poses[i] = new Pose2d(corners[i], new edu.wpi.first.math.geometry.Rotation2d());
        }
        poses[4] = poses[0]; // Close the loop

        return poses;
    }

    /**
     * Get number of obstacles on the field.
     * @return Number of obstacles
     */
    public int getObstacleCount() {
        return fieldMap.getObstacles().size();
    }

    /**
     * Get all obstacle poses for visualization.
     * @return Array of Pose2d arrays, one per obstacle
     */
    public Pose2d[][] getAllObstaclePoses() {
        int count = getObstacleCount();
        Pose2d[][] allObstacles = new Pose2d[count][];

        for (int i = 0; i < count; i++) {
            allObstacles[i] = getObstaclePoses(i);
        }

        return allObstacles;
    }

    /**
     * Publish obstacles to a Field2d as polyline outlines.
     * Uses Field2d.getObject().setPoses() to draw connected line segments.
     * Each obstacle is rendered as a closed polygon outline.
     *
     * Note: Visual styling (line color, width, arrows) must be configured
     * in Glass by right-clicking each object in the Field2d widget menu and
     * editing its properties. Recommended settings:
     *   - Line Color: #FFFF00 (yellow)
     *   - Line Weight: 3-4 pixels
     *   - Arrow Weight: 0 (no arrows)
     *
     * @param field The Field2d to publish to
     * @param currentRobotPose Current robot pose (unused, kept for API compatibility)
     */
    public void publishObstaclesToField2d(edu.wpi.first.wpilibj.smartdashboard.Field2d field, Pose2d currentRobotPose) {
        // Field boundary as closed rectangle outline
        field.getObject("FieldBoundary").setPoses(java.util.Arrays.asList(getFieldBoundaryPoses()));

        // Each obstacle as closed polygon outline
        for (int i = 0; i < getObstacleCount(); i++) {
            field.getObject("Obstacle/" + i).setPoses(java.util.Arrays.asList(getObstaclePoses(i)));
        }
    }


    // ==================== Collision Result Classes ====================

    /**
     * Result of a collision check.
     */
    public static class CollisionResult {
        public static final CollisionResult NO_COLLISION =
            new CollisionResult(false, CollisionType.NONE, null);

        public final boolean hasCollision;
        public final CollisionType type;
        public final CollisionGeometry.ConvexPolygon obstacle; // null if not obstacle collision

        public CollisionResult(boolean hasCollision, CollisionType type,
                              CollisionGeometry.ConvexPolygon obstacle) {
            this.hasCollision = hasCollision;
            this.type = type;
            this.obstacle = obstacle;
        }

        public boolean hasCollision() {
            return hasCollision;
        }
    }

    /**
     * Type of collision detected.
     */
    public enum CollisionType {
        NONE,
        FIELD_BOUNDARY,
        OBSTACLE
    }
}
