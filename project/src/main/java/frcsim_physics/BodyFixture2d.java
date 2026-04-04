package frcsim_physics;

/**
 *
 *
 * <h2>Body Fixture – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for {@code org.dyn4j.dynamics.BodyFixture}.
 *
 * <p>{@code IntakeSimulation} extends this class so that fixture identity comparisons work correctly.
 */
public class BodyFixture2d {
    public final Convex2d shape;
    private double friction = 0.5;
    private double restitution = 0.2;
    private double restitutionVelocity = 0.2;
    private double density = 1.0;

    public BodyFixture2d(Convex2d shape) {
        this.shape = shape;
    }

    public BodyFixture2d setFriction(double friction) {
        this.friction = friction;
        return this;
    }

    public BodyFixture2d setRestitution(double restitution) {
        this.restitution = restitution;
        return this;
    }

    public BodyFixture2d setRestitutionVelocity(double v) {
        this.restitutionVelocity = v;
        return this;
    }

    public BodyFixture2d setDensity(double density) {
        this.density = density;
        return this;
    }

    public double getFriction() {
        return friction;
    }

    public double getRestitution() {
        return restitution;
    }

    public double getRestitutionVelocity() {
        return restitutionVelocity;
    }

    public double getDensity() {
        return density;
    }
}
