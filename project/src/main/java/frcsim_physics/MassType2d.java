package frcsim_physics;

/**
 *
 *
 * <h2>Mass Type Enum – RenSim Physics Layer</h2>
 *
 * <p>Drop-in replacement for {@code org.dyn4j.geometry.MassType}.
 */
public enum MassType2d {
    /** Mass and inertia are computed from the body's fixtures. The body is fully dynamic. */
    NORMAL,
    /** Infinite mass – the body is static and cannot be moved by impulses. */
    INFINITE
}
