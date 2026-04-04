package frcsim_physics;

import java.util.ArrayList;
import java.util.List;

/**
 *
 *
 * <h2>Rigid Body – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for {@code org.dyn4j.dynamics.Body}.
 *
 * <p>Subclassed by {@code AbstractDriveTrainSimulation} and {@code GamePieceOnFieldSimulation}. The {@link #transform}
 * and {@link #linearVelocity} fields are public so that subclasses can mutate them directly, matching the dyn4j API.
 */
public class RigidBody {

    /** World-space transform – accessed directly by subclasses via {@code super.transform.set(...)}. */
    public final FrcTransform transform = new FrcTransform();

    /** World-space linear velocity – accessed directly by subclasses via {@code super.linearVelocity.set(...)}. */
    public final Vec2 linearVelocity = new Vec2();

    private double angularVelocity = 0.0;
    private final Vec2 forceAccum = new Vec2();
    private double torqueAccum = 0.0;

    private double linearDamping = 0.0;
    private double angularDamping = 0.0;
    private boolean bullet = false;
    private boolean isStatic = false;

    private double massKg = 1.0;
    private double inertia = 1.0;
    private double invMass = 1.0;
    private double invInertia = 1.0;

    protected final List<BodyFixture2d> fixtures = new ArrayList<>();

    // ── Fixture management ──────────────────────────────────────────────────

    public BodyFixture2d addFixture(Convex2d shape) {
        BodyFixture2d f = new BodyFixture2d(shape);
        fixtures.add(f);
        return f;
    }

    public BodyFixture2d addFixture(Convex2d shape, double density, double friction, double restitution) {
        BodyFixture2d f = new BodyFixture2d(shape);
        f.setDensity(density).setFriction(friction).setRestitution(restitution);
        fixtures.add(f);
        return f;
    }

    /** Adds an already-constructed fixture (e.g., an {@code IntakeSimulation} instance). */
    public void addFixture(BodyFixture2d fixture) {
        if (!fixtures.contains(fixture)) fixtures.add(fixture);
    }

    public boolean removeFixture(BodyFixture2d fixture) {
        return fixtures.remove(fixture);
    }

    public List<BodyFixture2d> getFixtures() {
        return fixtures;
    }

    // ── Mass ────────────────────────────────────────────────────────────────

    public void setMass(MassType2d type) {
        if (type == MassType2d.INFINITE) {
            isStatic = true;
            invMass = 0.0;
            invInertia = 0.0;
            massKg = Double.POSITIVE_INFINITY;
            inertia = Double.POSITIVE_INFINITY;
            return;
        }
        isStatic = false;
        double totalMass = 0.0, totalInertia = 0.0;
        for (BodyFixture2d f : fixtures) {
            double m = f.getDensity() * f.shape.getArea();
            totalMass += m;
            if (f.shape instanceof CircleShape cs) {
                // Solid disc: I = ½mr²
                totalInertia += 0.5 * m * cs.radius * cs.radius;
            } else if (f.shape instanceof RectangleShape rs) {
                // Solid rectangle: I = m(w²+h²)/12
                double hw = rs.getHalfWidth(), hh = rs.getHalfHeight();
                totalInertia += m * (hw * hw + hh * hh) / 3.0;
            } else {
                totalInertia += m * 0.1;
            }
        }
        massKg = totalMass > 1e-14 ? totalMass : 1.0;
        inertia = totalInertia > 1e-14 ? totalInertia : massKg * 0.1;
        invMass = 1.0 / massKg;
        invInertia = 1.0 / inertia;
    }

    public Mass2d getMass() {
        return new Mass2d(massKg, inertia);
    }

    public boolean isStatic() {
        return isStatic;
    }

    public double getInverseMass() {
        return invMass;
    }

    public double getInverseInertia() {
        return invInertia;
    }

    // ── Damping ─────────────────────────────────────────────────────────────

    public void setLinearDamping(double d) {
        linearDamping = d;
    }

    public void setAngularDamping(double d) {
        angularDamping = d;
    }

    public double getLinearDamping() {
        return linearDamping;
    }

    public double getAngularDamping() {
        return angularDamping;
    }

    // ── Transform ───────────────────────────────────────────────────────────

    public FrcTransform getTransform() {
        return transform;
    }

    /**
     * Overwrites the body's transform (position and rotation) with the given transform.
     *
     * <p>Unlike directly mutating {@link #transform}, this is the API-style setter used by
     * {@code GamePieceOnFieldSimulation} and mirrors dyn4j's {@code Body.setTransform(Transform)}.
     */
    public void setTransform(FrcTransform newTransform) {
        transform.set(newTransform);
    }

    // ── Velocity ────────────────────────────────────────────────────────────

    public Vec2 getLinearVelocity() {
        return linearVelocity.copy();
    }

    /**
     * Returns the velocity of a specific world-space point on the body, accounting for angular velocity.
     *
     * <p>{@code v_p = v_cm + ω × r} (2D: {@code v_p = v_cm + ω * perp(r)})
     */
    public Vec2 getLinearVelocity(Vec2 worldPoint) {
        Vec2 t = transform.getTranslation();
        double rx = worldPoint.x - t.x;
        double ry = worldPoint.y - t.y;
        return new Vec2(linearVelocity.x - angularVelocity * ry, linearVelocity.y + angularVelocity * rx);
    }

    public void setLinearVelocity(Vec2 v) {
        linearVelocity.set(v);
    }

    public void setLinearVelocity(double x, double y) {
        linearVelocity.set(x, y);
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    public void setAngularVelocity(double omega) {
        angularVelocity = omega;
    }

    // ── Forces ──────────────────────────────────────────────────────────────

    /** Applies a force at the centre of mass. */
    public void applyForce(Vec2 force) {
        forceAccum.add(force);
    }

    /**
     * Applies a force at a world-space point, generating both linear and angular impulse.
     *
     * <p>Torque: {@code τ = r × F}
     */
    public void applyForce(Vec2 force, Vec2 worldPoint) {
        forceAccum.add(force);
        Vec2 t = transform.getTranslation();
        double rx = worldPoint.x - t.x, ry = worldPoint.y - t.y;
        torqueAccum += rx * force.y - ry * force.x;
    }

    public void applyTorque(double torque) {
        torqueAccum += torque;
    }

    public Vec2 getForceAccumulator() {
        return forceAccum.copy();
    }

    // ── Integration ─────────────────────────────────────────────────────────

    /** Semi-implicit Euler velocity integration: {@code v += (F/m - damping*v) * dt}. */
    public void integrateVelocity(double dt) {
        if (isStatic) return;
        linearVelocity.x += (forceAccum.x * invMass - linearDamping * linearVelocity.x) * dt;
        linearVelocity.y += (forceAccum.y * invMass - linearDamping * linearVelocity.y) * dt;
        angularVelocity += (torqueAccum * invInertia - angularDamping * angularVelocity) * dt;
    }

    /** Integrates the body position and orientation using the current velocity. */
    public void integratePosition(double dt) {
        if (isStatic) return;
        Vec2 t = transform.getTranslation();
        transform.setTranslation(new Vec2(t.x + linearVelocity.x * dt, t.y + linearVelocity.y * dt));
        transform.setRotation(transform.getRotationRadians() + angularVelocity * dt);
    }

    public void clearForces() {
        forceAccum.set(0, 0);
        torqueAccum = 0;
    }

    // ── Geometry helpers ────────────────────────────────────────────────────

    /** Transforms a body-local point into world space using the current transform. */
    public Vec2 getWorldPoint(Vec2 localPoint) {
        return transform.transform(localPoint);
    }

    public void setBullet(boolean bullet) {
        this.bullet = bullet;
    }

    public boolean isBullet() {
        return bullet;
    }
}
