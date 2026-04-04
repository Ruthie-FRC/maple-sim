package frcsim_physics;

/**
 *
 *
 * <h2>SwerveModel – RenSim Swerve-Drive Body.</h2>
 *
 * <p>Models a swerve-drive robot as a single rigid body in the RenSim 3D physics world. Swerve
 * kinematics require tracking a 2D heading (yaw) separately from position, because RenSim's native
 * body only tracks 3D linear position and velocity.
 *
 * <p>Typical usage:
 *
 * <pre>{@code
 * PhysicsWorld world = new PhysicsWorld(0.005, false);
 * SwerveModel swerve = new SwerveModel(world, 50.0);
 * swerve.setChassisState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
 * swerve.step(1);
 * double[] pos = swerve.getPosition(); // {x, y, z}
 * }</pre>
 */
public final class SwerveModel {

    private final PhysicsWorld world;
    private final int bodyIndex;

    private double headingRadians = 0.0;
    private double omegaRadPs = 0.0;
    private final double fixedDtSeconds;

    /**
     * Creates a new swerve model attached to the given physics world.
     *
     * @param world the {@link PhysicsWorld} that owns the body
     * @param massKg total robot mass in kilograms (chassis + bumpers)
     */
    public SwerveModel(PhysicsWorld world, double massKg) {
        this.world = world;
        this.fixedDtSeconds = world.fixedDtSeconds();
        this.bodyIndex = world.createBody(massKg);
    }

    /**
     * Sets the full chassis state, pushing position and linear velocity into RenSim and storing
     * heading and angular velocity locally.
     *
     * @param xMeters robot x position in metres
     * @param yMeters robot y position in metres
     * @param headingRadians robot yaw angle in radians (CCW positive)
     * @param vxMps robot x velocity in m/s (field-relative)
     * @param vyMps robot y velocity in m/s (field-relative)
     * @param omegaRadPs robot angular velocity in rad/s (CCW positive)
     */
    public void setChassisState(
            double xMeters,
            double yMeters,
            double headingRadians,
            double vxMps,
            double vyMps,
            double omegaRadPs) {
        this.headingRadians = headingRadians;
        this.omegaRadPs = omegaRadPs;
        world.setBodyPosition(bodyIndex, xMeters, yMeters, 0.0);
        world.setBodyLinearVelocity(bodyIndex, vxMps, vyMps, 0.0);
    }

    /**
     * Returns the current 3D position of the robot body from RenSim, or {@code null} on failure.
     *
     * @return {@code double[3]} containing {x, y, z} in metres, or {@code null}
     */
    public double[] getPosition() {
        return world.bodyPosition(bodyIndex);
    }

    /**
     * Returns the current 3D linear velocity of the robot body from RenSim, or {@code null} on
     * failure.
     *
     * @return {@code double[3]} containing {vx, vy, vz} in m/s, or {@code null}
     */
    public double[] getLinearVelocity() {
        return world.bodyLinearVelocity(bodyIndex);
    }

    /**
     * Returns the current robot heading in radians (CCW positive, field-relative).
     *
     * @return heading in radians
     */
    public double getHeadingRadians() {
        return headingRadians;
    }

    /**
     * Returns the current robot angular velocity in rad/s (CCW positive).
     *
     * @return angular velocity in rad/s
     */
    public double getOmegaRadPs() {
        return omegaRadPs;
    }

    /**
     * Advances the simulation by the given number of fixed-timestep steps, then integrates heading
     * using the stored angular velocity.
     *
     * @param steps number of steps (must be ≥ 1)
     * @return true on success
     */
    public boolean step(int steps) {
        final int safeSteps = Math.max(steps, 1);
        final boolean ok = world.step(safeSteps);
        headingRadians += omegaRadPs * fixedDtSeconds * safeSteps;
        return ok;
    }

    /**
     * Returns the native RenSim body index for direct JNI access if needed.
     *
     * @return body index in the underlying {@link PhysicsWorld}
     */
    public int getBodyIndex() {
        return bodyIndex;
    }
}
