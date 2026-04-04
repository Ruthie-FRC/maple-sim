package frcsim_physics;

/**
 *
 *
 * <h2>Collision Data – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for {@code org.dyn4j.world.ContactCollisionData}. Carries the two bodies and their
 * respective fixtures involved in a contact event.
 */
public final class CollisionData2d {
    private final RigidBody body1;
    private final RigidBody body2;
    private final BodyFixture2d fixture1;
    private final BodyFixture2d fixture2;

    public CollisionData2d(
            RigidBody body1, BodyFixture2d fixture1, RigidBody body2, BodyFixture2d fixture2) {
        this.body1 = body1;
        this.fixture1 = fixture1;
        this.body2 = body2;
        this.fixture2 = fixture2;
    }

    public RigidBody getBody1() {
        return body1;
    }

    public RigidBody getBody2() {
        return body2;
    }

    public BodyFixture2d getFixture1() {
        return fixture1;
    }

    public BodyFixture2d getFixture2() {
        return fixture2;
    }
}
