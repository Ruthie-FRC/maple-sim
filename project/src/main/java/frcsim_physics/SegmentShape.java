package frcsim_physics;

/**
 *
 *
 * <h2>Line-Segment Collision Shape – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for dyn4j's segment shape created via {@code Geometry.createSegment(p1, p2)}.
 *
 * <p>Used for field boundary walls. Treated as a zero-thickness boundary that pushes dynamic bodies outward.
 */
public final class SegmentShape implements Convex2d {
    private final Vec2 p1Local;
    private final Vec2 p2Local;

    public SegmentShape(Vec2 p1Local, Vec2 p2Local) {
        this.p1Local = p1Local.copy();
        this.p2Local = p2Local.copy();
    }

    public Vec2 getP1World(FrcTransform bodyTransform) {
        return bodyTransform.transform(p1Local);
    }

    public Vec2 getP2World(FrcTransform bodyTransform) {
        return bodyTransform.transform(p2Local);
    }

    @Override
    public double getArea() {
        return 0.0;
    }

    @Override
    public boolean containsPoint(Vec2 worldPoint, FrcTransform bodyTransform) {
        return false;
    }

    /**
     * AABB expansion epsilon for segments (metres).
     *
     * <p>Segments have zero thickness, so their AABB would have zero area along the segment's normal, which would
     * cause the broad-phase check to miss edge-on contacts. This small expansion gives the AABB a non-zero depth.
     */
    private static final double AABB_EXPANSION = 0.05;

    @Override
    public double[] getWorldAabb(FrcTransform bodyTransform) {
        Vec2 w1 = getP1World(bodyTransform), w2 = getP2World(bodyTransform);
        return new double[] {
            Math.min(w1.x, w2.x) - AABB_EXPANSION,
            Math.min(w1.y, w2.y) - AABB_EXPANSION,
            Math.max(w1.x, w2.x) + AABB_EXPANSION,
            Math.max(w1.y, w2.y) + AABB_EXPANSION
        };
    }
}
