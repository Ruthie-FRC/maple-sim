package frcsim_physics;

/**
 *
 *
 * <h2>Physics Settings – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for {@code org.dyn4j.dynamics.Settings}. Stores tuning parameters for the physics world.
 */
public final class PhysicsSettings {
    private double minimumAtRestTime = 0.5;
    private int velocityConstraintIterations = 10;
    private int positionConstraintIterations = 3;

    public void setMinimumAtRestTime(double t) {
        minimumAtRestTime = t;
    }

    public double getMinimumAtRestTime() {
        return minimumAtRestTime;
    }

    public void setVelocityConstraintSolverIterations(int n) {
        velocityConstraintIterations = n;
    }

    public void setPositionConstraintSolverIterations(int n) {
        positionConstraintIterations = n;
    }

    public int getVelocityConstraintIterations() {
        return velocityConstraintIterations;
    }

    public int getPositionConstraintIterations() {
        return positionConstraintIterations;
    }
}
