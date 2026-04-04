package frcsim_physics;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 *
 *
 * <h2>2D Physics World – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for {@code org.dyn4j.world.World<Body>}.
 *
 * <p>Implements a simplified but correct 2D rigid-body physics world with:
 *
 * <ul>
 *   <li>Semi-implicit Euler integration (matching RenSim's default integration method)
 *   <li>SAT-based collision detection for OBB-OBB, OBB-circle, circle-circle, and segment collisions
 *   <li>Impulse-based collision resolution with restitution and position correction
 *   <li>Contact-listener callbacks used by {@code IntakeSimulation} to collect game pieces
 * </ul>
 *
 * <p>The static constant {@link #ZERO_GRAVITY} replaces {@code PhysicsWorld.ZERO_GRAVITY}.
 */
public final class PhysicsWorld2d {
    /** Zero-gravity constant – pass to {@link #setGravity(Vec2)} for a top-down simulation. */
    public static final Vec2 ZERO_GRAVITY = new Vec2(0, 0);

    private final List<RigidBody> bodies = new ArrayList<>();
    private final List<ContactListener2d> contactListeners = new ArrayList<>();
    private Vec2 gravity = ZERO_GRAVITY;
    private PhysicsSettings settings = new PhysicsSettings();

    // ── World management ────────────────────────────────────────────────────

    public void addBody(RigidBody body) {
        if (!bodies.contains(body)) bodies.add(body);
    }

    public boolean removeBody(RigidBody body) {
        return bodies.remove(body);
    }

    public void removeAllBodies() {
        bodies.clear();
    }

    public void setGravity(Vec2 g) {
        gravity = g;
    }

    public Vec2 getGravity() {
        return gravity.copy();
    }

    public void addContactListener(ContactListener2d listener) {
        contactListeners.add(listener);
    }

    public PhysicsSettings getSettings() {
        return settings;
    }

    public void setSettings(PhysicsSettings s) {
        this.settings = s;
    }

    // ── Simulation step ─────────────────────────────────────────────────────

    /**
     * Advances the simulation by one step.
     *
     * @param iterations unused (kept for API compatibility with dyn4j)
     * @param dt sub-tick duration in seconds
     */
    public void step(int iterations, double dt) {
        // 1. Integrate velocities (apply forces + damping)
        for (RigidBody body : bodies) {
            if (body.isStatic()) continue;
            // Apply gravity (zero in top-down FRC simulation)
            body.applyForce(new Vec2(gravity.x * body.getMass().getMass(), gravity.y * body.getMass().getMass()));
            body.integrateVelocity(dt);
        }

        // 2. Detect collisions, notify listeners, resolve impulses
        detectAndResolve(dt);

        // 3. Integrate positions
        for (RigidBody body : bodies) {
            body.integratePosition(dt);
        }

        // 4. Clear force accumulators
        for (RigidBody body : bodies) {
            body.clearForces();
        }
    }

    // ── Collision pipeline ──────────────────────────────────────────────────

    private void detectAndResolve(double dt) {
        int n = bodies.size();
        for (int i = 0; i < n; i++) {
            RigidBody a = bodies.get(i);
            for (int j = i + 1; j < n; j++) {
                RigidBody b = bodies.get(j);
                if (a.isStatic() && b.isStatic()) continue;
                resolveBodyPair(a, b, dt);
            }
        }
    }

    private void resolveBodyPair(RigidBody a, RigidBody b, double dt) {
        // Broad phase: body-level AABB check
        double[] aabbA = getBodyAabb(a);
        double[] aabbB = getBodyAabb(b);
        if (!aabbOverlap(aabbA, aabbB)) return;

        // Narrow phase: check each fixture pair
        for (BodyFixture2d fa : a.getFixtures()) {
            for (BodyFixture2d fb : b.getFixtures()) {
                Manifold m = computeManifold(a, fa, b, fb);
                if (m == null) continue;

                // Notify contact listeners (for intake detection)
                CollisionData2d cd = new CollisionData2d(a, fa, b, fb);
                Contact2d contact = new Contact2d();
                for (ContactListener2d listener : contactListeners) {
                    listener.begin(cd, contact);
                }

                // Apply impulse resolution
                applyImpulse(a, fa, b, fb, m);

                // Position correction (Baumgarte)
                correctPositions(a, b, m);
            }
        }
    }

    // ── AABB helpers ────────────────────────────────────────────────────────

    private static double[] getBodyAabb(RigidBody body) {
        double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
        for (BodyFixture2d f : body.getFixtures()) {
            double[] bb = f.shape.getWorldAabb(body.transform);
            minX = Math.min(minX, bb[0]);
            minY = Math.min(minY, bb[1]);
            maxX = Math.max(maxX, bb[2]);
            maxY = Math.max(maxY, bb[3]);
        }
        return new double[] {minX, minY, maxX, maxY};
    }

    private static boolean aabbOverlap(double[] a, double[] b) {
        return a[0] <= b[2] && a[2] >= b[0] && a[1] <= b[3] && a[3] >= b[1];
    }

    // ── Collision manifold ──────────────────────────────────────────────────

    /** Holds the contact normal, penetration depth, and contact point for a collision. */
    private static final class Manifold {
        Vec2 normal; // points from B toward A
        double penetration;
        Vec2 contactPoint;

        Manifold(Vec2 normal, double penetration, Vec2 contactPoint) {
            this.normal = normal;
            this.penetration = penetration;
            this.contactPoint = contactPoint;
        }
    }

    private static Manifold computeManifold(RigidBody a, BodyFixture2d fa, RigidBody b, BodyFixture2d fb) {
        Convex2d sa = fa.shape, sb = fb.shape;

        if (sa instanceof CircleShape ca && sb instanceof CircleShape cb)
            return circleCircle(a.transform, ca, b.transform, cb);
        if (sa instanceof CircleShape ca && sb instanceof RectangleShape rb)
            return circleRect(a.transform, ca, b.transform, rb);
        if (sa instanceof RectangleShape ra && sb instanceof CircleShape cb)
            return flipManifold(circleRect(b.transform, cb, a.transform, ra));
        if (sa instanceof RectangleShape ra && sb instanceof RectangleShape rb)
            return rectRect(a.transform, ra, b.transform, rb);
        if (sa instanceof SegmentShape ss && sb instanceof CircleShape cb)
            return segmentCircle(a.transform, ss, b.transform, cb);
        if (sa instanceof CircleShape ca && sb instanceof SegmentShape ss)
            return flipManifold(segmentCircle(b.transform, ss, a.transform, ca));
        if (sa instanceof SegmentShape ss && sb instanceof RectangleShape rb)
            return segmentRect(a.transform, ss, b.transform, rb);
        if (sa instanceof RectangleShape ra && sb instanceof SegmentShape ss)
            return flipManifold(segmentRect(b.transform, ss, a.transform, ra));

        return null;
    }

    private static Manifold flipManifold(Manifold m) {
        if (m == null) return null;
        return new Manifold(m.normal.copy().negate(), m.penetration, m.contactPoint);
    }

    // ── Circle-circle ───────────────────────────────────────────────────────

    private static Manifold circleCircle(FrcTransform ta, CircleShape ca, FrcTransform tb, CircleShape cb) {
        Vec2 cA = ca.getWorldCenter(ta), cB = cb.getWorldCenter(tb);
        double dx = cA.x - cB.x, dy = cA.y - cB.y;
        double dist = Math.sqrt(dx * dx + dy * dy);
        double radSum = ca.radius + cb.radius;
        if (dist >= radSum) return null;
        Vec2 normal = dist > 1e-10 ? new Vec2(dx / dist, dy / dist) : new Vec2(1, 0);
        Vec2 cp = new Vec2(cB.x + normal.x * cb.radius, cB.y + normal.y * cb.radius);
        return new Manifold(normal, radSum - dist, cp);
    }

    // ── Circle-rectangle ────────────────────────────────────────────────────

    private static Manifold circleRect(FrcTransform ta, CircleShape ca, FrcTransform tb, RectangleShape rb) {
        Vec2 circleWorld = ca.getWorldCenter(ta);
        // Transform circle centre into rectangle's local frame
        Vec2 circleLocal = tb.inverseTransform(circleWorld);
        double[] obb = rb.getWorldObb(tb); // use local half-extents directly
        double hw = rb.getHalfWidth(), hh = rb.getHalfHeight();
        Vec2 offset = rb.getLocalOffset();
        double lx = circleLocal.x - offset.x;
        double ly = circleLocal.y - offset.y;
        // Rotate into shape frame
        double cos = Math.cos(-rb.getLocalRotation()), sin = Math.sin(-rb.getLocalRotation());
        double sx = cos * lx - sin * ly;
        double sy = sin * lx + cos * ly;
        // Closest point on rectangle to circle centre (in shape frame)
        double cx = Math.max(-hw, Math.min(hw, sx));
        double cy = Math.max(-hh, Math.min(hh, sy));
        double ddx = sx - cx, ddy = sy - cy;
        double dist2 = ddx * ddx + ddy * ddy;
        if (dist2 >= ca.radius * ca.radius) return null;
        double dist = Math.sqrt(dist2);
        // Normal in shape frame → world frame
        Vec2 normalShape = dist > 1e-10 ? new Vec2(ddx / dist, ddy / dist) : new Vec2(1, 0);
        // Rotate normal from shape local to body local then to world
        double totalRot = tb.getRotationRadians() + rb.getLocalRotation();
        double cosT = Math.cos(totalRot), sinT = Math.sin(totalRot);
        Vec2 normal = new Vec2(
                cosT * normalShape.x - sinT * normalShape.y,
                sinT * normalShape.x + cosT * normalShape.y);
        Vec2 cp = new Vec2(circleWorld.x - normal.x * ca.radius, circleWorld.y - normal.y * ca.radius);
        return new Manifold(normal, ca.radius - dist, cp);
    }

    // ── Rectangle-rectangle (SAT) ────────────────────────────────────────────

    private static Manifold rectRect(FrcTransform ta, RectangleShape ra, FrcTransform tb, RectangleShape rb) {
        double[] obbA = ra.getWorldObb(ta);
        double[] obbB = rb.getWorldObb(tb);
        double angA = obbA[2], angB = obbB[2];
        double cosA = Math.cos(angA), sinA = Math.sin(angA);
        double cosB = Math.cos(angB), sinB = Math.sin(angB);
        Vec2[] axes = {
            new Vec2(cosA, sinA), new Vec2(-sinA, cosA),
            new Vec2(cosB, sinB), new Vec2(-sinB, cosB)
        };
        Vec2[] cornersA = ra.getWorldCorners(ta);
        Vec2[] cornersB = rb.getWorldCorners(tb);

        double minPen = Double.POSITIVE_INFINITY;
        Vec2 bestAxis = null;
        for (Vec2 axis : axes) {
            double[] projA = projectCorners(cornersA, axis);
            double[] projB = projectCorners(cornersB, axis);
            double overlap = Math.min(projA[1], projB[1]) - Math.max(projA[0], projB[0]);
            if (overlap <= 0) return null;
            if (overlap < minPen) {
                minPen = overlap;
                bestAxis = axis;
            }
        }
        // Orient normal from B toward A
        Vec2 d = new Vec2(obbA[0] - obbB[0], obbA[1] - obbB[1]);
        if (d.dot(bestAxis) < 0) bestAxis = bestAxis.copy().negate();
        Vec2 cp = new Vec2((obbA[0] + obbB[0]) / 2, (obbA[1] + obbB[1]) / 2);
        return new Manifold(bestAxis.copy().normalize(), minPen, cp);
    }

    private static double[] projectCorners(Vec2[] corners, Vec2 axis) {
        double min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
        for (Vec2 c : corners) {
            double p = c.dot(axis);
            min = Math.min(min, p);
            max = Math.max(max, p);
        }
        return new double[] {min, max};
    }

    // ── Segment-circle ───────────────────────────────────────────────────────

    private static Manifold segmentCircle(
            FrcTransform ta, SegmentShape seg, FrcTransform tb, CircleShape circle) {
        Vec2 p1 = seg.getP1World(ta), p2 = seg.getP2World(ta);
        Vec2 cc = circle.getWorldCenter(tb);
        Vec2 closest = closestPointOnSegment(p1, p2, cc);
        double dx = cc.x - closest.x, dy = cc.y - closest.y;
        double dist2 = dx * dx + dy * dy;
        if (dist2 >= circle.radius * circle.radius) return null;
        double dist = Math.sqrt(dist2);
        Vec2 normal = dist > 1e-10 ? new Vec2(dx / dist, dy / dist) : segNormal(p1, p2);
        return new Manifold(normal, circle.radius - dist, closest);
    }

    private static Vec2 closestPointOnSegment(Vec2 p1, Vec2 p2, Vec2 pt) {
        double ex = p2.x - p1.x, ey = p2.y - p1.y;
        double len2 = ex * ex + ey * ey;
        if (len2 < 1e-14) return p1.copy();
        double t = Math.max(0, Math.min(1, ((pt.x - p1.x) * ex + (pt.y - p1.y) * ey) / len2));
        return new Vec2(p1.x + t * ex, p1.y + t * ey);
    }

    private static Vec2 segNormal(Vec2 p1, Vec2 p2) {
        double ex = p2.x - p1.x, ey = p2.y - p1.y;
        double len = Math.sqrt(ex * ex + ey * ey);
        return len > 1e-14 ? new Vec2(-ey / len, ex / len) : new Vec2(0, 1);
    }

    // ── Segment-rectangle ────────────────────────────────────────────────────

    private static Manifold segmentRect(
            FrcTransform ta, SegmentShape seg, FrcTransform tb, RectangleShape rect) {
        Vec2 p1 = seg.getP1World(ta), p2 = seg.getP2World(ta);
        Vec2[] corners = rect.getWorldCorners(tb);
        // Check all corners against the segment
        Vec2 sn = segNormal(p1, p2);
        Vec2 segDir = new Vec2(p2.x - p1.x, p2.y - p1.y).normalize();
        double minPen = Double.POSITIVE_INFINITY;
        Vec2 cp = null;
        for (Vec2 corner : corners) {
            Vec2 closest = closestPointOnSegment(p1, p2, corner);
            double dx = corner.x - closest.x, dy = corner.y - closest.y;
            double dist = Math.sqrt(dx * dx + dy * dy);
            if (dist < 0.02 && dist < minPen) {
                minPen = dist;
                cp = closest;
            }
        }
        if (cp == null) return null;
        Vec2 rectCenter = tb.transform(rect.getLocalOffset());
        Vec2 d = new Vec2(rectCenter.x - cp.x, rectCenter.y - cp.y);
        Vec2 normal = sn.dot(d) > 0 ? sn.copy() : sn.copy().negate();
        return new Manifold(normal, 0.02 - minPen, cp);
    }

    // ── Impulse resolution ───────────────────────────────────────────────────

    private static void applyImpulse(
            RigidBody a, BodyFixture2d fa, RigidBody b, BodyFixture2d fb, Manifold m) {
        Vec2 n = m.normal;
        Vec2 cp = m.contactPoint;
        Vec2 rA = new Vec2(cp.x - a.transform.getTranslation().x, cp.y - a.transform.getTranslation().y);
        Vec2 rB = new Vec2(cp.x - b.transform.getTranslation().x, cp.y - b.transform.getTranslation().y);

        Vec2 vA = a.getLinearVelocity(cp);
        Vec2 vB = b.getLinearVelocity(cp);
        Vec2 relVel = new Vec2(vB.x - vA.x, vB.y - vA.y);
        double closingSpeed = relVel.dot(n);
        if (closingSpeed > 0) return; // bodies are separating

        double e = Math.min(fa.getRestitution(), fb.getRestitution());
        double velThreshold = Math.max(fa.getRestitutionVelocity(), fb.getRestitutionVelocity());
        if (Math.abs(closingSpeed) < velThreshold) e = 0.0;

        double rACrossN = rA.cross(n);
        double rBCrossN = rB.cross(n);
        double denom = a.getInverseMass() + b.getInverseMass()
                + rACrossN * rACrossN * a.getInverseInertia()
                + rBCrossN * rBCrossN * b.getInverseInertia();
        if (denom < 1e-14) return;

        double j = -(1.0 + e) * closingSpeed / denom;

        if (!a.isStatic()) {
            a.setLinearVelocity(new Vec2(
                    a.getLinearVelocity().x - j * a.getInverseMass() * n.x,
                    a.getLinearVelocity().y - j * a.getInverseMass() * n.y));
            a.setAngularVelocity(a.getAngularVelocity() - j * a.getInverseInertia() * rA.cross(n));
        }
        if (!b.isStatic()) {
            b.setLinearVelocity(new Vec2(
                    b.getLinearVelocity().x + j * b.getInverseMass() * n.x,
                    b.getLinearVelocity().y + j * b.getInverseMass() * n.y));
            b.setAngularVelocity(b.getAngularVelocity() + j * b.getInverseInertia() * rB.cross(n));
        }
    }

    // ── Position correction (Baumgarte) ──────────────────────────────────────

    private static void correctPositions(RigidBody a, RigidBody b, Manifold m) {
        final double SLOP = 0.005; // penetration allowed before correction
        final double BETA = 0.3; // fraction of penetration to correct per step
        double pen = Math.max(m.penetration - SLOP, 0.0);
        double totalInvMass = a.getInverseMass() + b.getInverseMass();
        if (totalInvMass < 1e-14) return;
        double correctionMag = pen * BETA / totalInvMass;
        Vec2 correction = new Vec2(correctionMag * m.normal.x, correctionMag * m.normal.y);
        if (!a.isStatic()) {
            Vec2 ta = a.transform.getTranslation();
            a.transform.setTranslation(new Vec2(
                    ta.x - a.getInverseMass() * correction.x,
                    ta.y - a.getInverseMass() * correction.y));
        }
        if (!b.isStatic()) {
            Vec2 tb = b.transform.getTranslation();
            b.transform.setTranslation(new Vec2(
                    tb.x + b.getInverseMass() * correction.x,
                    tb.y + b.getInverseMass() * correction.y));
        }
    }
}
