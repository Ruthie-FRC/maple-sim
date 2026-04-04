package frcsim_physics;

/**
 *
 *
 * <h2>Contact Listener – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for {@code org.dyn4j.world.listener.ContactListener<Body>}. Implemented by
 * {@code IntakeSimulation.GamePieceContactListener} to detect when game pieces touch the intake.
 */
public interface ContactListener2d {
    void begin(CollisionData2d collision, Contact2d contact);

    void persist(CollisionData2d collision, Contact2d oldContact, Contact2d newContact);

    void end(CollisionData2d collision, Contact2d contact);

    void destroyed(CollisionData2d collision, Contact2d contact);

    void collision(CollisionData2d collision);

    void preSolve(CollisionData2d collision, Contact2d contact);

    void postSolve(CollisionData2d collision, SolvedContact2d contact);
}
