package frcsim_physics;

/**
 *
 *
 * <h2>Rectangle Collision Shape – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for {@code org.dyn4j.geometry.Rectangle}.
 *
 * <p>Supports a local offset and local rotation, set at construction time via {@link #rotate(double)} and
 * {@link #translate(Vec2)}. These are applied relative to the owning body's transform when computing world positions.
 */
public final class RectangleShape implements Convex2d {
    private final double halfWidth;
    private final double halfHeight;
    private final Vec2 localOffset = new Vec2();
    private double localRotation = 0.0;

    public RectangleShape(double width, double height) {
        this.halfWidth = width / 2.0;
        this.halfHeight = height / 2.0;
    }

    /** Rotates the shape's local orientation by the given angle (radians). */
    public RectangleShape rotate(double angle) {
        localRotation += angle;
        return this;
    }

    /** Translates the shape's local centre. */
    public RectangleShape translate(Vec2 offset) {
        localOffset.add(offset);
        return this;
    }

    public double getHalfWidth() {
        return halfWidth;
    }

    public double getHalfHeight() {
        return halfHeight;
    }

    public Vec2 getLocalOffset() {
        return localOffset.copy();
    }

    public double getLocalRotation() {
        return localRotation;
    }

    /**
     * Checks whether the given world-space point lies inside this rectangle (no body transform applied, i.e., the
     * rectangle is assumed to be already positioned in world space via {@link #translate(Vec2)} and
     * {@link #rotate(double)}).
     *
     * <p>Alias for {@link #containsPoint(Vec2, FrcTransform)} with an identity transform.
     */
    public boolean contains(Vec2 worldPoint) {
        return containsPoint(worldPoint, new FrcTransform());
    }

    @Override
    public double getArea() {
        return 4.0 * halfWidth * halfHeight;
    }

    /** Checks whether the given world-space point lies inside this rectangle. */
    @Override
    public boolean containsPoint(Vec2 worldPoint, FrcTransform bodyTransform) {
        Vec2 localPt = bodyTransform.inverseTransform(worldPoint);
        double dx = localPt.x - localOffset.x;
        double dy = localPt.y - localOffset.y;
        double cos = Math.cos(-localRotation), sin = Math.sin(-localRotation);
        double shapeX = cos * dx - sin * dy;
        double shapeY = sin * dx + cos * dy;
        return Math.abs(shapeX) <= halfWidth && Math.abs(shapeY) <= halfHeight;
    }

    /**
     * Returns the oriented bounding box (OBB) parameters in world space.
     *
     * @return {@code [cx, cy, totalRotation, halfWidth, halfHeight]}
     */
    public double[] getWorldObb(FrcTransform bodyTransform) {
        Vec2 wc = bodyTransform.transform(localOffset);
        double wRot = bodyTransform.getRotationRadians() + localRotation;
        return new double[] {wc.x, wc.y, wRot, halfWidth, halfHeight};
    }

    @Override
    public double[] getWorldAabb(FrcTransform bodyTransform) {
        double[] obb = getWorldObb(bodyTransform);
        double cos = Math.abs(Math.cos(obb[2])), sin = Math.abs(Math.sin(obb[2]));
        double extX = cos * halfWidth + sin * halfHeight;
        double extY = sin * halfWidth + cos * halfHeight;
        return new double[] {obb[0] - extX, obb[1] - extY, obb[0] + extX, obb[1] + extY};
    }

    /** Returns the 4 world-space corners of this rectangle (CCW order). */
    public Vec2[] getWorldCorners(FrcTransform bodyTransform) {
        double[] obb = getWorldObb(bodyTransform);
        double cx = obb[0], cy = obb[1], ang = obb[2];
        double cos = Math.cos(ang), sin = Math.sin(ang);
        double[] xs = {halfWidth, halfWidth, -halfWidth, -halfWidth};
        double[] ys = {halfHeight, -halfHeight, -halfHeight, halfHeight};
        Vec2[] corners = new Vec2[4];
        for (int i = 0; i < 4; i++) {
            corners[i] = new Vec2(cx + cos * xs[i] - sin * ys[i], cy + sin * xs[i] + cos * ys[i]);
        }
        return corners;
    }
}
