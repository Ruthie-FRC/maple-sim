package frcsim_physics;

/**
 *
 *
 * <h2>2-Dimensional Vector – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for {@code org.dyn4j.geometry.Vector2}, following the RenSim frcsim_physics API.
 */
public final class Vec2 {
    public double x;
    public double y;

    public Vec2() {
        this.x = 0;
        this.y = 0;
    }

    public Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Creates a unit vector pointing at the given angle (radians). Mirrors the dyn4j {@code Vector2(double angle)}
     * constructor.
     */
    public Vec2(double angle) {
        this.x = Math.cos(angle);
        this.y = Math.sin(angle);
    }

    public Vec2(Vec2 other) {
        this.x = other.x;
        this.y = other.y;
    }

    /** Creates a vector from polar coordinates. */
    public static Vec2 create(double magnitude, double angle) {
        return new Vec2(magnitude * Math.cos(angle), magnitude * Math.sin(angle));
    }

    public Vec2 set(double x, double y) {
        this.x = x;
        this.y = y;
        return this;
    }

    public Vec2 set(Vec2 other) {
        this.x = other.x;
        this.y = other.y;
        return this;
    }

    public Vec2 add(Vec2 other) {
        x += other.x;
        y += other.y;
        return this;
    }

    public Vec2 add(double ax, double ay) {
        x += ax;
        y += ay;
        return this;
    }

    public Vec2 subtract(Vec2 other) {
        x -= other.x;
        y -= other.y;
        return this;
    }

    public Vec2 multiply(double scalar) {
        x *= scalar;
        y *= scalar;
        return this;
    }

    public Vec2 negate() {
        x = -x;
        y = -y;
        return this;
    }

    public Vec2 copy() {
        return new Vec2(x, y);
    }

    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public double getMagnitudeSquared() {
        return x * x + y * y;
    }

    public double getDirection() {
        return Math.atan2(y, x);
    }

    public double dot(Vec2 other) {
        return x * other.x + y * other.y;
    }

    /** 2D cross product – returns the scalar z-component. */
    public double cross(Vec2 other) {
        return x * other.y - y * other.x;
    }

    public Vec2 normalize() {
        double mag = getMagnitude();
        if (mag > 1e-14) {
            x /= mag;
            y /= mag;
        }
        return this;
    }

    public Vec2 getNormalized() {
        return copy().normalize();
    }

    /** Returns the unsigned angle (radians) between this vector and {@code other}, in [0, π]. */
    public double getAngleBetween(Vec2 other) {
        double mag = getMagnitude() * other.getMagnitude();
        if (mag < 1e-14) return 0.0;
        return Math.acos(Math.max(-1.0, Math.min(1.0, dot(other) / mag)));
    }

    /** Rotates this vector in-place by the given angle (radians). */
    public Vec2 rotate(double angle) {
        double cos = Math.cos(angle), sin = Math.sin(angle);
        double nx = x * cos - y * sin;
        double ny = x * sin + y * cos;
        x = nx;
        y = ny;
        return this;
    }

    @Override
    public String toString() {
        return "Vec2(" + x + ", " + y + ")";
    }
}
