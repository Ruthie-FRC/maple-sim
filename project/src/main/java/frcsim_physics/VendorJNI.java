package frcsim_physics;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 *
 *
 * <h2>VendorJNI – RenSim Native Bridge.</h2>
 *
 * <p>JNI entry point that bridges Java to the RenSim C physics driver ({@code libVendorDriver}).
 * All methods are {@code static native}; they delegate directly to the C API declared in {@code
 * driverheader.h}.
 *
 * <p>If the native library cannot be loaded (e.g., running on a platform without the compiled
 * shared library), the load failure is caught and {@link #nativeLibraryAvailable} is set to {@code
 * false}. All callers should check this flag before invoking native methods.
 */
public class VendorJNI {
    private VendorJNI() {}

    /** {@code true} if {@code libVendorDriver} was loaded successfully. */
    public static boolean nativeLibraryAvailable = false;

    /** Controls whether the library is extracted and loaded on static initialisation. */
    public static class Helper {
        private Helper() {}

        private static AtomicBoolean extractOnStaticLoad = new AtomicBoolean(true);

        public static boolean getExtractOnStaticLoad() {
            return extractOnStaticLoad.get();
        }

        public static void setExtractOnStaticLoad(boolean load) {
            extractOnStaticLoad.set(load);
        }
    }

    static {
        if (Helper.getExtractOnStaticLoad()) {
            try {
                System.loadLibrary("VendorDriver");
                nativeLibraryAvailable = true;
            } catch (UnsatisfiedLinkError ignored) {
                // Native library not available; fall back to pure-Java simulation.
            }
        }
    }

    /**
     * Forces an immediate attempt to load the native library, regardless of the {@link
     * Helper#getExtractOnStaticLoad()} flag. Safe to call multiple times; subsequent calls are
     * no-ops if the library is already loaded.
     */
    public static synchronized void forceLoad() {
        if (nativeLibraryAvailable) return;
        try {
            System.loadLibrary("VendorDriver");
            nativeLibraryAvailable = true;
        } catch (UnsatisfiedLinkError ignored) {
            // Native library still unavailable.
        }
    }

    // ── Native method declarations ───────────────────────────────────────────

    /**
     * Performs one-time initialization of the native driver.
     *
     * <p>Must be called before any other method when the native library is loaded. Returns {@code
     * 1} on success; any other value indicates an initialization failure.
     *
     * @return {@code 1} on success
     */
    public static native int initialize();

    /**
     * Creates a native physics world.
     *
     * <p>Every successful call to {@code createWorld} must be paired with a corresponding {@link
     * #destroyWorld(long)} call to release native resources. The returned handle is {@code 0} on
     * failure and non-zero on success.
     *
     * @param fixedDtSeconds the fixed simulation timestep in seconds (e.g. {@code 0.005} for 200 Hz)
     * @param enableGravity {@code true} to enable gravity; {@code false} for top-down FRC field simulations
     * @return a non-zero native world handle on success, or {@code 0} on failure
     */
    public static native long createWorld(double fixedDtSeconds, boolean enableGravity);

    /**
     * Destroys a previously created native physics world and releases all associated resources.
     *
     * <p>Must be called once for every successful {@link #createWorld(double, boolean)} call.
     * Passing an invalid or already-destroyed handle is a no-op.
     *
     * @param worldHandle the native world handle returned by {@link #createWorld(double, boolean)}
     */
    public static native void destroyWorld(long worldHandle);

    public static native int createBody(long worldHandle, double massKg);

    public static native int setBodyPosition(
            long worldHandle, int bodyIndex, double xMeters, double yMeters, double zMeters);

    public static native int setBodyLinearVelocity(
            long worldHandle, int bodyIndex, double vxMps, double vyMps, double vzMps);

    public static native int setBodyGravityEnabled(
            long worldHandle, int bodyIndex, boolean enabled);

    public static native int setWorldGravity(
            long worldHandle, double gxMps2, double gyMps2, double gzMps2);

    public static native int stepWorld(long worldHandle, int steps);

    public static native int getBodyPosition(
            long worldHandle, int bodyIndex, double[] outXyzMeters);

    public static native int getBodyLinearVelocity(
            long worldHandle, int bodyIndex, double[] outVxyzMps);
}
