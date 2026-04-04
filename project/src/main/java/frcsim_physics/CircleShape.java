package frcsim_physics;

/**
 *
 *
 * <h2>Circle Collision Shape – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for {@code org.dyn4j.geometry.Circle}.
 */
public final class CircleShape implements Convex2d {
    public final double radius;
    private final Vec2 localOffset;

    public CircleShape(double radius) {
        this.radius = radius;
        this.localOffset = new Vec2();
    }

    public Vec2 getLocalOffset() {
        return localOffset.copy();
    }

    public Vec2 getWorldCenter(FrcTransform bodyTransform) {
        return bodyTransform.transform(localOffset);
    }

    @Override
    public double getArea() {
        return Math.PI * radius * radius;
    }

    @Override
    public boolean containsPoint(Vec2 worldPoint, FrcTransform bodyTransform) {
        Vec2 center = getWorldCenter(bodyTransform);
        double dx = worldPoint.x - center.x, dy = worldPoint.y - center.y;
        return dx * dx + dy * dy <= radius * radius;
    }

    @Override
    public double[] getWorldAabb(FrcTransform bodyTransform) {
        Vec2 c = getWorldCenter(bodyTransform);
        return new double[] {c.x - radius, c.y - radius, c.x + radius, c.y + radius};
    }
}
