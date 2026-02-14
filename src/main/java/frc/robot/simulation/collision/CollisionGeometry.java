package frc.robot.simulation.collision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Collision geometry primitives for simulation collision detection.
 * Provides AABB (Axis-Aligned Bounding Box) optimization and SAT (Separating Axis Theorem) collision detection.
 */
public class CollisionGeometry {

    // ==================== Axis-Aligned Bounding Box ====================

    /**
     * Axis-Aligned Bounding Box for fast rejection tests.
     * Used as Phase 1 of two-phase collision detection.
     */
    public static class AABB {
        public final double minX;
        public final double maxX;
        public final double minY;
        public final double maxY;

        public AABB(double minX, double maxX, double minY, double maxY) {
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
        }

        /**
         * Fast O(1) AABB overlap test.
         * @param other The other AABB to test against
         * @return true if AABBs overlap, false otherwise
         */
        public boolean overlaps(AABB other) {
            return !(maxX < other.minX || minX > other.maxX ||
                     maxY < other.minY || minY > other.maxY);
        }

        /**
         * Check if this AABB fully contains another AABB.
         * @param other The AABB to test containment for
         * @return true if other is fully contained within this AABB
         */
        public boolean contains(AABB other) {
            return other.minX >= minX && other.maxX <= maxX &&
                   other.minY >= minY && other.maxY <= maxY;
        }

        /**
         * Check if a point is contained within this AABB.
         * @param point The point to test
         * @return true if point is inside the AABB
         */
        public boolean contains(Translation2d point) {
            return point.getX() >= minX && point.getX() <= maxX &&
                   point.getY() >= minY && point.getY() <= maxY;
        }

        /**
         * Create an AABB from a set of points.
         * @param points The points to compute the AABB from
         * @return AABB that bounds all the points
         */
        public static AABB fromPoints(Translation2d... points) {
            if (points.length == 0) {
                return new AABB(0, 0, 0, 0);
            }

            double minX = points[0].getX();
            double maxX = points[0].getX();
            double minY = points[0].getY();
            double maxY = points[0].getY();

            for (int i = 1; i < points.length; i++) {
                minX = Math.min(minX, points[i].getX());
                maxX = Math.max(maxX, points[i].getX());
                minY = Math.min(minY, points[i].getY());
                maxY = Math.max(maxY, points[i].getY());
            }

            return new AABB(minX, maxX, minY, maxY);
        }
    }

    // ==================== Collision Shape Interface ====================

    /**
     * Interface for all collision shapes.
     */
    public interface CollisionShape {
        /**
         * Get the axis-aligned bounding box for this shape.
         * Used for fast rejection in Phase 1 of collision detection.
         * @return The bounding box
         */
        AABB getBoundingBox();

        /**
         * Test intersection with another collision shape.
         * This is the precise Phase 2 collision test (SAT).
         * @param other The other shape to test against
         * @return true if shapes intersect, false otherwise
         */
        boolean intersects(CollisionShape other);
    }

    // ==================== Robot Bounds ====================

    /**
     * Represents the robot as an oriented rectangle.
     * The robot can be rotated, so AABB must account for all possible corner positions.
     */
    public static class RobotBounds implements CollisionShape {
        private final double lengthMeters;  // X-axis dimension (front-to-back)
        private final double widthMeters;   // Y-axis dimension (side-to-side)
        private Pose2d centerPose;          // Robot center position and rotation

        public RobotBounds(double lengthMeters, double widthMeters) {
            this.lengthMeters = lengthMeters;
            this.widthMeters = widthMeters;
            this.centerPose = new Pose2d(); // Default at origin
        }

        /**
         * Create a new RobotBounds at a specific pose.
         * @param pose The pose to position the robot at
         * @return New RobotBounds positioned at the given pose
         */
        public RobotBounds at(Pose2d pose) {
            RobotBounds bounds = new RobotBounds(lengthMeters, widthMeters);
            bounds.centerPose = pose;
            return bounds;
        }

        /**
         * Get the four corners of the robot in field coordinates.
         * Accounts for robot rotation.
         * @return Array of 4 corner positions [FL, FR, BR, BL]
         */
        public Translation2d[] getCorners() {
            double halfLength = lengthMeters / 2.0;
            double halfWidth = widthMeters / 2.0;

            // Corners in robot-relative coordinates
            Translation2d[] localCorners = {
                new Translation2d(halfLength, halfWidth),   // Front-left
                new Translation2d(halfLength, -halfWidth),  // Front-right
                new Translation2d(-halfLength, -halfWidth), // Back-right
                new Translation2d(-halfLength, halfWidth)   // Back-left
            };

            // Transform to field coordinates
            Translation2d[] fieldCorners = new Translation2d[4];
            for (int i = 0; i < 4; i++) {
                fieldCorners[i] = centerPose.getTranslation().plus(
                    localCorners[i].rotateBy(centerPose.getRotation())
                );
            }

            return fieldCorners;
        }

        @Override
        public AABB getBoundingBox() {
            // Compute AABB that contains all rotated corners
            return AABB.fromPoints(getCorners());
        }

        @Override
        public boolean intersects(CollisionShape other) {
            if (other instanceof ConvexPolygon) {
                // Convert robot to polygon and use SAT
                ConvexPolygon robotPolygon = new ConvexPolygon(getCorners());
                return robotPolygon.intersects(other);
            }
            return false;
        }

        public double getLengthMeters() {
            return lengthMeters;
        }

        public double getWidthMeters() {
            return widthMeters;
        }

        public Pose2d getCenterPose() {
            return centerPose;
        }
    }

    // ==================== Field Boundary ====================

    /**
     * Represents the field boundary as an axis-aligned rectangle.
     * This is already axis-aligned, so AABB equals the boundary itself.
     */
    public static class FieldBoundary implements CollisionShape {
        private final AABB bounds;

        public FieldBoundary(double minX, double maxX, double minY, double maxY) {
            this.bounds = new AABB(minX, maxX, minY, maxY);
        }

        /**
         * Check if robot bounds are fully contained within the field.
         * @param robot The robot bounds to check
         * @return true if robot is fully inside field boundaries
         */
        public boolean contains(RobotBounds robot) {
            // Check all corners are within field
            Translation2d[] corners = robot.getCorners();
            for (Translation2d corner : corners) {
                if (!bounds.contains(corner)) {
                    return false;
                }
            }
            return true;
        }

        /**
         * Clamp a point to be within the field boundaries.
         * @param point The point to clamp
         * @return Clamped point inside field boundaries
         */
        public Translation2d clampPoint(Translation2d point) {
            double clampedX = Math.max(bounds.minX, Math.min(bounds.maxX, point.getX()));
            double clampedY = Math.max(bounds.minY, Math.min(bounds.maxY, point.getY()));
            return new Translation2d(clampedX, clampedY);
        }

        @Override
        public AABB getBoundingBox() {
            return bounds;
        }

        @Override
        public boolean intersects(CollisionShape other) {
            // Field boundary doesn't intersect - we check containment instead
            return false;
        }

        public double getMinX() { return bounds.minX; }
        public double getMaxX() { return bounds.maxX; }
        public double getMinY() { return bounds.minY; }
        public double getMaxY() { return bounds.maxY; }
    }

    // ==================== Convex Polygon ====================

    /**
     * Represents a convex polygon obstacle.
     * Uses Separating Axis Theorem (SAT) for precise collision detection.
     */
    public static class ConvexPolygon implements CollisionShape {
        private final Translation2d[] vertices;
        private final AABB cachedBoundingBox;

        public ConvexPolygon(Translation2d... vertices) {
            this.vertices = vertices;
            this.cachedBoundingBox = AABB.fromPoints(vertices);
        }

        @Override
        public AABB getBoundingBox() {
            return cachedBoundingBox;
        }

        @Override
        public boolean intersects(CollisionShape other) {
            if (other instanceof ConvexPolygon) {
                return satIntersects((ConvexPolygon) other);
            } else if (other instanceof RobotBounds) {
                // Convert robot to polygon
                RobotBounds robot = (RobotBounds) other;
                ConvexPolygon robotPolygon = new ConvexPolygon(robot.getCorners());
                return satIntersects(robotPolygon);
            }
            return false;
        }

        /**
         * Separating Axis Theorem (SAT) collision detection.
         * Tests if two convex polygons intersect.
         * @param other The other polygon to test against
         * @return true if polygons intersect, false if separated
         */
        private boolean satIntersects(ConvexPolygon other) {
            // Test all axes from both polygons
            List<Translation2d> axes = new ArrayList<>();
            axes.addAll(getEdgeNormals(this));
            axes.addAll(getEdgeNormals(other));

            for (Translation2d axis : axes) {
                Range range1 = projectOntoAxis(this, axis);
                Range range2 = projectOntoAxis(other, axis);

                if (!range1.overlaps(range2)) {
                    // Found separating axis - no collision
                    return false;
                }
            }

            // No separating axis found - collision exists
            return true;
        }

        /**
         * Get edge normal vectors for SAT.
         * @param polygon The polygon to get normals from
         * @return List of perpendicular edge normal vectors
         */
        private List<Translation2d> getEdgeNormals(ConvexPolygon polygon) {
            List<Translation2d> normals = new ArrayList<>();

            for (int i = 0; i < polygon.vertices.length; i++) {
                Translation2d p1 = polygon.vertices[i];
                Translation2d p2 = polygon.vertices[(i + 1) % polygon.vertices.length];

                // Edge vector
                Translation2d edge = p2.minus(p1);

                // Perpendicular normal (rotate 90 degrees)
                Translation2d normal = new Translation2d(-edge.getY(), edge.getX());

                // Normalize
                double length = normal.getNorm();
                if (length > 0.0001) {
                    normal = normal.div(length);
                    normals.add(normal);
                }
            }

            return normals;
        }

        /**
         * Project polygon onto an axis and return the min/max range.
         * @param polygon The polygon to project
         * @param axis The axis to project onto
         * @return Range of projection
         */
        private Range projectOntoAxis(ConvexPolygon polygon, Translation2d axis) {
            double min = axis.getX() * polygon.vertices[0].getX() +
                        axis.getY() * polygon.vertices[0].getY();
            double max = min;

            for (int i = 1; i < polygon.vertices.length; i++) {
                double projection = axis.getX() * polygon.vertices[i].getX() +
                                  axis.getY() * polygon.vertices[i].getY();
                min = Math.min(min, projection);
                max = Math.max(max, projection);
            }

            return new Range(min, max);
        }

        public Translation2d[] getVertices() {
            return vertices;
        }
    }

    // ==================== Helper Classes ====================

    /**
     * Represents a 1D range for SAT projection tests.
     */
    private static class Range {
        final double min;
        final double max;

        Range(double min, double max) {
            this.min = min;
            this.max = max;
        }

        boolean overlaps(Range other) {
            return !(max < other.min || min > other.max);
        }
    }
}
