package frcsim_physics;

/**
 *
 *
 * <h2>Convex 2D Shape Interface – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for {@code org.dyn4j.geometry.Convex}.
 */
public interface Convex2d {
    /** Returns the area of this shape in m². Used for density calculations. */
    double getArea();

    /** Returns true if the given world-space point lies inside this shape. */
    boolean containsPoint(Vec2 worldPoint, FrcTransform bodyTransform);

    /**
     * Returns the axis-aligned bounding box of this shape in world space.
     *
     * @return {@code [minX, minY, maxX, maxY]}
     */
    double[] getWorldAabb(FrcTransform bodyTransform);
}
