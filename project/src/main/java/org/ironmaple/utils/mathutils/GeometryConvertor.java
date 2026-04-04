package org.ironmaple.utils.mathutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frcsim_physics.FrcRotation;
import frcsim_physics.FrcTransform;
import frcsim_physics.Vec2;

/** Utils to convert between WPILib and the RenSim frcsim_physics geometry classes. */
public class GeometryConvertor {
    public static Vec2 toDyn4jVector2(Translation2d wpilibTranslation2d) {
        return new Vec2(wpilibTranslation2d.getX(), wpilibTranslation2d.getY());
    }

    public static Translation2d toWpilibTranslation2d(Vec2 vec2) {
        return new Translation2d(vec2.x, vec2.y);
    }

    public static FrcRotation toDyn4jRotation(Rotation2d wpilibRotation2d) {
        return new FrcRotation(wpilibRotation2d.getRadians());
    }

    public static Rotation2d toWpilibRotation2d(FrcRotation frcRotation) {
        return new Rotation2d(frcRotation.toRadians());
    }

    public static FrcTransform toDyn4jTransform(Pose2d wpilibPose2d) {
        final FrcTransform transform = new FrcTransform();
        transform.setTranslation(toDyn4jVector2(wpilibPose2d.getTranslation()));
        transform.setRotation(toDyn4jRotation(wpilibPose2d.getRotation()));
        return transform;
    }

    public static Pose2d toWpilibPose2d(FrcTransform frcTransform) {
        return new Pose2d(
                toWpilibTranslation2d(frcTransform.getTranslation()),
                toWpilibRotation2d(frcTransform.getRotation()));
    }

    public static Vec2 toDyn4jLinearVelocity(ChassisSpeeds wpilibChassisSpeeds) {
        return new Vec2(wpilibChassisSpeeds.vxMetersPerSecond, wpilibChassisSpeeds.vyMetersPerSecond);
    }

    public static ChassisSpeeds toWpilibChassisSpeeds(Vec2 linearVelocity, double angularVelocityRadPerSec) {
        return new ChassisSpeeds(linearVelocity.x, linearVelocity.y, angularVelocityRadPerSec);
    }

    public static Translation2d getChassisSpeedsTranslationalComponent(ChassisSpeeds chassisSpeeds) {
        return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }
}
