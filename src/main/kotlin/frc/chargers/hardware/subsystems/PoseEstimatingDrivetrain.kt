package frc.chargers.hardware.subsystems

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Time
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.wpilibextensions.Rotation2d
import frc6995.NomadAprilTagUtil
import limelight.LimelightHelpers
import monologue.Logged
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import kotlin.jvm.optionals.getOrNull

/**
 * A base class for a drivetrain that can estimate its own pose,
 * and fuse pose measurements from one or more vision cameras.
 */
@Suppress("unused")
abstract class PoseEstimatingDrivetrain: SubsystemBase(), Logged {
    abstract val robotPose: Pose2d

    abstract fun resetPose(pose: Pose2d = Pose2d())

    abstract fun addVisionMeasurement(pose: Pose2d, timestamp: Time, stdDevs: Matrix<N3, N1>? = null)

    private val visionUpdateCallbacks = mutableListOf<() -> Unit>()

    /**
     * Adds a vision pose measurement; calculating pose standard deviation from camera yaw.
     */
    fun addVisionMeasurement(pose: Pose2d, timestamp: Time, cameraYaw: Angle){
        addVisionMeasurement(
            pose, timestamp,
            NomadAprilTagUtil.calculateVisionUncertainty(pose.x, pose.rotation, Rotation2d(cameraYaw))
        )
    }

    /**
     * Registers a photon camera for vision pose estimation.
     */
    fun addPhotonPoseSupplier(
        photonCam: PhotonCamera,
        robotToCamera: Transform3d,
        poseStrategy: PoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    ){
        val poseEstimator = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
            poseStrategy,
            photonCam,
            robotToCamera
        )
        val camYaw = robotToCamera.rotation.x.ofUnit(radians)

        visionUpdateCallbacks.add {
            val poseEstimation = poseEstimator.update().getOrNull()
            if (poseEstimation == null) {
                log("PhotonPoseEstimations/${photonCam.name}", Pose3d())
            } else {
                log("PhotonPoseEstimations/${photonCam.name}", poseEstimation.estimatedPose)
                addVisionMeasurement(
                    poseEstimation.estimatedPose.toPose2d(),
                    poseEstimation.timestampSeconds.ofUnit(seconds),
                    camYaw
                )
            }
        }
    }

    /**
     * Registers a limelight for vision pose estimation.
     *
     * Precondition: If useMegaTag2 is enabled, you must call LimelightHelpers.setRobotOrientation periodically.
     * This can be accomplished via gyro.broadcastOrientationForMegaTag2(),
     * where the gyro can be a [frc.chargers.hardware.imu.ChargerNavX] or a [frc.chargers.hardware.imu.ChargerPigeon2].
     */
    fun addLimelightPoseSupplier(
        camName: String,
        robotToCamera: Transform3d,
        useMegaTag2: Boolean
    ){
        LimelightHelpers.setCameraPose_RobotSpace(
            camName,
            robotToCamera.x,
            robotToCamera.y,
            robotToCamera.z,
            robotToCamera.rotation.x,
            robotToCamera.rotation.y,
            robotToCamera.rotation.z
        )
        val camYaw = robotToCamera.rotation.z.ofUnit(radians)

        visionUpdateCallbacks.add {
            val poseEstimation: LimelightHelpers.PoseEstimate
            if (useMegaTag2){
                LimelightHelpers.setRobotOrientation(
                    camName, this.robotPose.rotation.degrees,
                    0.0, 0.0, 0.0, 0.0, 0.0
                )
                poseEstimation = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camName)
            }else{
                poseEstimation = LimelightHelpers.getBotPoseEstimate_wpiBlue(camName)
            }

            val tagEstimateAmbiguous = poseEstimation.rawFiducials.size == 1 &&
                poseEstimation.rawFiducials[0].ambiguity >= 0.9

            if (poseEstimation.tagCount > 0 && !tagEstimateAmbiguous){
                addVisionMeasurement(
                    poseEstimation.pose,
                    poseEstimation.timestampSeconds.ofUnit(seconds),
                    camYaw
                )
                log("LimelightPoseEstimations/$camName", poseEstimation.pose)
            }else{
                log("LimelightPoseEstimations/$camName", Pose2d())
            }
        }
    }

    override fun periodic() {
        visionUpdateCallbacks.forEach { it() }
    }
}