package frcsim_physics;

/**
 *
 *
 * <h2>Mass – RenSim Physics Layer</h2>
 *
 * <p>Returned by {@link RigidBody#getMass()}. Provides access to the body's inertia, which is needed by
 * {@code SwerveDriveSimulation} for angular acceleration calculations.
 */
public final class Mass2d {
    private final double mass;
    private final double inertia;

    public Mass2d(double mass, double inertia) {
        this.mass = mass;
        this.inertia = inertia;
    }

    public double getMass() {
        return mass;
    }

    public double getInertia() {
        return inertia;
    }
}
