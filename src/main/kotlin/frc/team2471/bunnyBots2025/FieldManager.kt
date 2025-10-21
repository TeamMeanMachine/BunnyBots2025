package frc.team2471.bunnyBots2025

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.meters
import org.team2471.frc.lib.units.UTranslation2d
import org.team2471.frc.lib.units.wrap
import org.team2471.frc.lib.util.isRedAlliance

object FieldManager {
    val aprilTagFieldLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
    val allAprilTags = aprilTagFieldLayout.tags

    val fieldWidth = aprilTagFieldLayout.fieldWidth.meters
    val fieldLength = aprilTagFieldLayout.fieldLength.meters

    val fieldDimensions = UTranslation2d(fieldLength, fieldWidth)

    val fieldHalfWidth = fieldWidth / 2.0
    val fieldHalfLength = fieldLength / 2.0

    val fieldCenter = fieldDimensions / 2.0


    /**
     * Reflects [Translation2d] across the midline of the field. Useful for mirrored field layouts (2023, 2024).
     * Units must be meters
     * @param doReflect Supplier to perform reflection. Default: true
     * @see Translation2d.rotateAroundField
     */
    fun Translation2d.reflectAcrossField(doReflect: () -> Boolean = { true }): Translation2d {
        return if (doReflect()) Translation2d(fieldLength.asMeters - x, y) else this
    }

    /**
     * Reflects [Pose2d] across the midline of the field. Useful for mirrored field layouts (2023, 2024).
     * Units must be meters
     * @param doReflect Supplier to perform reflection. Default: true
     * @see Pose2d.rotateAroundField
     */
    fun Pose2d.reflectAcrossField(doReflect: () -> Boolean = { true }): Pose2d {
        return if (doReflect()) Pose2d(fieldLength.asMeters - x, y, (rotation - 180.0.degrees.asRotation2d).wrap()) else this
    }

    /**
     * Rotates the [Translation2d] 180 degrees around the center of the field. Useful for reflected field layouts (2022, 2025).
     * Units must be meters
     * @param doRotate Supplier to perform rotation. Default: true
     * @see Translation2d.reflectAcrossField
     */
    fun Translation2d.rotateAroundField(doRotate: () -> Boolean = { true }): Translation2d {
        return if (doRotate()) this.rotateAround(fieldCenter, 180.0.degrees.asRotation2d) else this
    }

    /**
     * Rotates the [Pose2d] 180 degrees around the center of the field. Useful for reflected field layouts (2022, 2025).
     * Units must be meters
     * @param doRotate Supplier to perform rotation. Default: true
     * @see Pose2d.reflectAcrossField
     */
    fun Pose2d.rotateAroundField(doRotate: () -> Boolean = { true }): Pose2d {
        return if (doRotate()) this.rotateAround(fieldCenter, 180.0.degrees.asRotation2d) else this
    }

    /**
     * Returns if the [Translation2d] is on the red alliance side of the field.
     */
    fun Translation2d.onRedSide(): Boolean = this.x > fieldCenter.x.asMeters
    /**
     * Returns if the [Translation2d] is on the blue alliance side of the field.
     */
    fun Translation2d.onBlueSide(): Boolean = !this.onRedSide()
    /**
     * Returns if the [Translation2d] is closer to your current alliance's side of the field.
     */
    fun Translation2d.onFriendlyAllianceSide() = this.onRedSide() == isRedAlliance
    /**
     * Returns if the [Translation2d] is closer to your opponent alliance's side of the field.
     */
    fun Translation2d.onOpposingAllianceSide() = !this.onFriendlyAllianceSide()

    /**
     * Returns if the [Pose2d] is on the red alliance side of the field.
     */
    fun Pose2d.onRedSide(): Boolean = this.translation.onRedSide()
    /**
     * Returns if the [Pose2d] is on the blue alliance side of the field.
     */
    fun Pose2d.onBlueSide(): Boolean = !this.onRedSide()
    /**
     * Returns if the [Pose2d] is closer to your current alliance's side of the field.
     */
    fun Pose2d.onFriendlyAllianceSide() = this.translation.onFriendlyAllianceSide()
    /**
     * Returns if the [Pose2d] is closer to your opponent alliance's side of the field.
     */
    fun Pose2d.onOpposingAllianceSide() = !this.onFriendlyAllianceSide()

}
