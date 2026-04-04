package frcsim_physics;

/**
 *
 *
 * <h2>2D Rotation – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for {@code org.dyn4j.geometry.Rotation}.
 */
public final class FrcRotation {
    private double radians;

    public FrcRotation() {
        this.radians = 0;
    }

    public FrcRotation(double radians) {
        this.radians = radians;
    }

    public double toRadians() {
        return radians;
    }

    public double toDegrees() {
        return Math.toDegrees(radians);
    }

    public static FrcRotation fromRadians(double radians) {
        return new FrcRotation(radians);
    }

    public static FrcRotation fromDegrees(double degrees) {
        return new FrcRotation(Math.toRadians(degrees));
    }
}
