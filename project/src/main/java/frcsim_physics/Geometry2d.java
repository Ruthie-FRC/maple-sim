package frcsim_physics;

/**
 *
 *
 * <h2>Shape Factory – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for {@code org.dyn4j.geometry.Geometry} factory methods.
 */
public final class Geometry2d {
    private Geometry2d() {}

    public static CircleShape createCircle(double radius) {
        return new CircleShape(radius);
    }

    public static RectangleShape createRectangle(double width, double height) {
        return new RectangleShape(width, height);
    }

    public static SegmentShape createSegment(Vec2 p1, Vec2 p2) {
        return new SegmentShape(p1, p2);
    }
}
