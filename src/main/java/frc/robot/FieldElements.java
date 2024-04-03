package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

/** Enum to keep track of field element positions when using AprilTags.
 * All positions are created from Blue Side Origin (Bottom Left = (0, 0))
 * Allows for easy switch of alliance side targetting.
 * For example, if targetting the blue speaker, and then targetting red speaker next match, pose is automatically mirrored. */
public enum FieldElements
{
    // Field Elements Defined on Blue Side Origin, Mirrored in getPose() method based on DriverStation info.
    SPEAKER(new Pose3d(0.2, 5.55, 2.045, new Rotation3d()), 7)
    , AMP(new Pose3d(1.84, 8.14, 0, new Rotation3d())) // TODO: Find Target Height
    , CLOSE_NOTE(new Pose3d(15, 0.5, 0, new Rotation3d()))
    , MIDDLE_NOTE(new Pose3d(15.5, 0.8, 0, new Rotation3d()))
    , FAR_NOTE(new Pose3d(16.1, 1.15, 0, new Rotation3d()));

    // TODO: Find Measurements for Stage Targetting
    // STAGE1(new Pose3d(4.85, 4.1, 0, new Rotation3d()));
    // STAGE2(new Pose3d(0, 0, 0, new Rotation3d()));
    // STAGE3(new Pose3d(0, 0, 0, new Rotation3d()));

    
    private Pose3d fieldElementPosition = null;
    private int aprilTagTargetID = 0;

    FieldElements(Pose3d fieldElementPosition)
    {
        this.fieldElementPosition = fieldElementPosition;
    }

    FieldElements(Pose3d fieldElementPosition, int aprilTagTargetID)
    {
        this.fieldElementPosition = fieldElementPosition;
        this.aprilTagTargetID = aprilTagTargetID;
    }


    /**
     * Blue Alliance being the origin,
     * @return whether field element positions should be mirrored to opposite side of field to target opposite alliance field elements.
     */
    private boolean shouldMirrorFieldElement()
    {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }


    /**
     * @return The Pose3d of field element including field mirroring.
     */
    public Pose3d getPose() {
        if (shouldMirrorFieldElement())
            // return fieldElementPosition.getTranslation().
            return fieldElementPosition.transformBy(new Transform3d((16.54 - (fieldElementPosition.getX()*2)), 0, 0, new Rotation3d()));
        else
            return fieldElementPosition;
    }

    public int getAprilTagTarget()
    {
        switch (aprilTagTargetID) {
            case 7:
                if (shouldMirrorFieldElement())
                {
                    return 4;
                }
                return 7;
            default:
                return aprilTagTargetID;
        }
    }
}