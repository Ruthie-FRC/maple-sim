package org.ruthiefrc.rensim.utils.mathutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frcsim_physics.FrcRotation;
import frcsim_physics.FrcTransform;
import frcsim_physics.Vec2;

/** Utils to convert between WPILib and the RenSim frcsim_physics geometry classes. */
public class GeometryConvertor {
    /** Converts a WPILib {@link Translation2d} to a frcsim_physics {@link Vec2}. */
    public static Vec2 toVec2(Translation2d wpilibTranslation2d) {
        return new Vec2(wpilibTranslation2d.getX(), wpilibTranslation2d.getY());
    }

    /** Backward-compatible alias for {@link #toVec2(Translation2d)}.
     *
     * @deprecated Use {@link #toVec2(Translation2d)} — dyn4j is no longer used.
     */
    @Deprecated
    public static Vec2 toDyn4jVector2(Translation2d wpilibTranslation2d) {
        return toVec2(wpilibTranslation2d);
    }

    /** Converts a frcsim_physics {@link Vec2} to a WPILib {@link Translation2d}. */
    public static Translation2d toWpilibTranslation2d(Vec2 vec2) {
        return new Translation2d(vec2.x, vec2.y);
    }

    /** Converts a WPILib {@link Rotation2d} to a frcsim_physics {@link FrcRotation}. */
    public static FrcRotation toFrcRotation(Rotation2d wpilibRotation2d) {
        return new FrcRotation(wpilibRotation2d.getRadians());
    }

    /** Backward-compatible alias for {@link #toFrcRotation(Rotation2d)}.
     *
     * @deprecated Use {@link #toFrcRotation(Rotation2d)} — dyn4j is no longer used.
     */
    @Deprecated
    public static FrcRotation toDyn4jRotation(Rotation2d wpilibRotation2d) {
        return toFrcRotation(wpilibRotation2d);
    }

    /** Converts a frcsim_physics {@link FrcRotation} to a WPILib {@link Rotation2d}. */
    public static Rotation2d toWpilibRotation2d(FrcRotation frcRotation) {
        return new Rotation2d(frcRotation.toRadians());
    }

    /** Converts a WPILib {@link Pose2d} to a frcsim_physics {@link FrcTransform}. */
    public static FrcTransform toFrcTransform(Pose2d wpilibPose2d) {
        final FrcTransform transform = new FrcTransform();
        transform.setTranslation(toVec2(wpilibPose2d.getTranslation()));
        transform.setRotation(toFrcRotation(wpilibPose2d.getRotation()));
        return transform;
    }

    /** Backward-compatible alias for {@link #toFrcTransform(Pose2d)}.
     *
     * @deprecated Use {@link #toFrcTransform(Pose2d)} — dyn4j is no longer used.
     */
    @Deprecated
    public static FrcTransform toDyn4jTransform(Pose2d wpilibPose2d) {
        return toFrcTransform(wpilibPose2d);
    }

    /** Converts a frcsim_physics {@link FrcTransform} to a WPILib {@link Pose2d}. */
    public static Pose2d toWpilibPose2d(FrcTransform frcTransform) {
        return new Pose2d(
                toWpilibTranslation2d(frcTransform.getTranslation()),
                toWpilibRotation2d(frcTransform.getRotation()));
    }

    /**
     * Extracts the translational component of a {@link ChassisSpeeds} as a frcsim_physics {@link Vec2}, suitable for
     * use as a linear-velocity vector in the physics engine.
     */
    public static Vec2 toLinearVelocityVec2(ChassisSpeeds wpilibChassisSpeeds) {
        return new Vec2(wpilibChassisSpeeds.vxMetersPerSecond, wpilibChassisSpeeds.vyMetersPerSecond);
    }

    /** Backward-compatible alias for {@link #toLinearVelocityVec2(ChassisSpeeds)}.
     *
     * @deprecated Use {@link #toLinearVelocityVec2(ChassisSpeeds)} — dyn4j is no longer used.
     */
    @Deprecated
    public static Vec2 toDyn4jLinearVelocity(ChassisSpeeds wpilibChassisSpeeds) {
        return toLinearVelocityVec2(wpilibChassisSpeeds);
    }

    /** Constructs a {@link ChassisSpeeds} from a linear-velocity {@link Vec2} and an angular velocity. */
    public static ChassisSpeeds toWpilibChassisSpeeds(Vec2 linearVelocity, double angularVelocityRadPerSec) {
        return new ChassisSpeeds(linearVelocity.x, linearVelocity.y, angularVelocityRadPerSec);
    }

    /** Returns the translational component of a {@link ChassisSpeeds} as a WPILib {@link Translation2d}. */
    public static Translation2d getChassisSpeedsTranslationalComponent(ChassisSpeeds chassisSpeeds) {
        return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }
}
