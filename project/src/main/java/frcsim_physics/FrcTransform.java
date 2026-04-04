package frcsim_physics;

/**
 *
 *
 * <h2>2D Transform – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for {@code org.dyn4j.geometry.Transform}.
 *
 * <p>The {@code translation} and {@code rotation} state is exposed through getters/setters; the {@link RigidBody}
 * class holds this object as a public field so subclasses can call {@code super.transform.set(...)} directly.
 */
public final class FrcTransform {
    private final Vec2 translation = new Vec2();
    private double rotation = 0.0; // radians

    public FrcTransform() {}

    public Vec2 getTranslation() {
        return translation.copy();
    }

    public FrcRotation getRotation() {
        return new FrcRotation(rotation);
    }

    public double getRotationRadians() {
        return rotation;
    }

    public void setTranslation(Vec2 t) {
        translation.set(t);
    }

    public void setRotation(FrcRotation r) {
        rotation = r.toRadians();
    }

    public void setRotation(double radians) {
        rotation = radians;
    }

    public void set(FrcTransform other) {
        translation.set(other.translation);
        rotation = other.rotation;
    }

    /** Transforms a local-space point into world space. */
    public Vec2 transform(Vec2 localPoint) {
        double cos = Math.cos(rotation), sin = Math.sin(rotation);
        return new Vec2(
                cos * localPoint.x - sin * localPoint.y + translation.x,
                sin * localPoint.x + cos * localPoint.y + translation.y);
    }

    /** Inverse transform: converts a world-space point into body-local space. */
    public Vec2 inverseTransform(Vec2 worldPoint) {
        double dx = worldPoint.x - translation.x;
        double dy = worldPoint.y - translation.y;
        double cos = Math.cos(rotation), sin = Math.sin(rotation);
        return new Vec2(cos * dx + sin * dy, -sin * dx + cos * dy);
    }
}
