package frcsim_physics;

/**
 *
 *
 * <h2>RenSim Physics World – Java binding.</h2>
 *
 * <p>Java-side representation of the RenSim C++ physics world. Wraps {@link VendorJNI} to provide
 * a clean, object-oriented interface identical to the C++ {@code rensim::PhysicsWorld} class
 * declared in {@code header.h}.
 *
 * <p>This class implements {@link AutoCloseable}; always use it in a try-with-resources block or
 * call {@link #close()} explicitly to release the native world handle.
 */
public final class PhysicsWorld implements AutoCloseable {

    private long worldHandle;
    private final double fixedDtSeconds;
    private boolean closed = false;

    /**
     * Creates a new physics world.
     *
     * @param fixedDtSeconds the fixed simulation timestep in seconds (e.g. 0.005 for 200 Hz)
     * @param enableGravity true to enable gravity; false for top-down (FRC field) simulations
     */
    public PhysicsWorld(double fixedDtSeconds, boolean enableGravity) {
        this.fixedDtSeconds = fixedDtSeconds;
        if (VendorJNI.nativeLibraryAvailable) {
            worldHandle = VendorJNI.createWorld(fixedDtSeconds, enableGravity);
        } else {
            worldHandle = -1L;
        }
    }

    /**
     * Creates a new top-down (zero-gravity) physics world suitable for FRC field simulation, with a
     * 5 ms fixed timestep.
     */
    public PhysicsWorld() {
        this(0.005, false);
    }

    /**
     * Creates a body in this world and returns its native index.
     *
     * @param massKg body mass in kilograms
     * @return the native body index (pass to set/get methods), or -1 on failure
     */
    public int createBody(double massKg) {
        if (worldHandle < 0) return -1;
        return VendorJNI.createBody(worldHandle, massKg);
    }

    /**
     * Sets a body's 3D position.
     *
     * @param bodyIndex the native body index
     * @param xMeters x position in metres
     * @param yMeters y position in metres
     * @param zMeters z position in metres
     * @return true on success
     */
    public boolean setBodyPosition(int bodyIndex, double xMeters, double yMeters, double zMeters) {
        return worldHandle >= 0
                && VendorJNI.setBodyPosition(worldHandle, bodyIndex, xMeters, yMeters, zMeters)
                        == 0;
    }

    /**
     * Sets a body's linear velocity.
     *
     * @param bodyIndex the native body index
     * @param vxMps x velocity in m/s
     * @param vyMps y velocity in m/s
     * @param vzMps z velocity in m/s
     * @return true on success
     */
    public boolean setBodyLinearVelocity(
            int bodyIndex, double vxMps, double vyMps, double vzMps) {
        return worldHandle >= 0
                && VendorJNI.setBodyLinearVelocity(worldHandle, bodyIndex, vxMps, vyMps, vzMps)
                        == 0;
    }

    /**
     * Enables or disables gravity for an individual body.
     *
     * @param bodyIndex the native body index
     * @param enabled true to enable gravity for this body
     * @return true on success
     */
    public boolean setBodyGravityEnabled(int bodyIndex, boolean enabled) {
        return worldHandle >= 0
                && VendorJNI.setBodyGravityEnabled(worldHandle, bodyIndex, enabled) == 0;
    }

    /**
     * Sets the world gravity vector.
     *
     * @param gxMps2 x gravity component in m/s²
     * @param gyMps2 y gravity component in m/s²
     * @param gzMps2 z gravity component in m/s² (typically −9.80665 for downward)
     * @return true on success
     */
    public boolean setGravity(double gxMps2, double gyMps2, double gzMps2) {
        return worldHandle >= 0
                && VendorJNI.setWorldGravity(worldHandle, gxMps2, gyMps2, gzMps2) == 0;
    }

    /**
     * Advances the simulation by the given number of fixed-timestep steps.
     *
     * @param steps number of steps (must be ≥ 1)
     * @return true on success
     */
    public boolean step(int steps) {
        return worldHandle >= 0 && VendorJNI.stepWorld(worldHandle, Math.max(steps, 1)) == 0;
    }

    /**
     * Returns a body's current 3D position, or {@code null} on failure.
     *
     * @param bodyIndex the native body index
     * @return {@code double[3]} containing {x, y, z} in metres, or {@code null}
     */
    public double[] bodyPosition(int bodyIndex) {
        if (worldHandle < 0) return null;
        final double[] out = new double[3];
        return VendorJNI.getBodyPosition(worldHandle, bodyIndex, out) == 0 ? out : null;
    }

    /**
     * Returns a body's current linear velocity, or {@code null} on failure.
     *
     * @param bodyIndex the native body index
     * @return {@code double[3]} containing {vx, vy, vz} in m/s, or {@code null}
     */
    public double[] bodyLinearVelocity(int bodyIndex) {
        if (worldHandle < 0) return null;
        final double[] out = new double[3];
        return VendorJNI.getBodyLinearVelocity(worldHandle, bodyIndex, out) == 0 ? out : null;
    }

    /**
     * Returns true if the native world was successfully created.
     *
     * @return true if the native world handle is valid
     */
    public boolean isValid() {
        return worldHandle >= 0;
    }

    /**
     * Returns the fixed simulation timestep in seconds.
     *
     * @return fixed timestep in seconds
     */
    public double fixedDtSeconds() {
        return fixedDtSeconds;
    }

    @Override
    public void close() {
        if (!closed && worldHandle >= 0) {
            VendorJNI.destroyWorld(worldHandle);
            worldHandle = -1L;
            closed = true;
        }
    }
}
