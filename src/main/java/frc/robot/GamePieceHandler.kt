//package frc.robot
//
//import edu.wpi.first.math.geometry.Pose2d
//import edu.wpi.first.math.geometry.Rotation2d
//import edu.wpi.first.math.geometry.Translation2d
//import edu.wpi.first.wpilibj2.command.Commands
//import frc.chargers.utils.Tunables.TunableBoolean
//import frc.chargers.utils.Tunables.TunableDouble
//import org.ironmaple.simulation.SimulatedArena
//import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation
//import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField
//import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoral
//import java.util.function.Function
//
//object GamePieceHandler {
//    private val simArena = SimulatedArena.getInstance()
//
//    private fun addTunableCoral(
//        name: String,
//        initialPose: Pose2d
//    ) {
//        var previousGp = ReefscapeCoral(initialPose)
//        val x = TunableDouble("$name/xMeters", initialPose.x)
//        val y = TunableDouble("$name/yMeters", initialPose.y)
//        val rotation = TunableDouble("$name/rotationDeg", initialPose.rotation.degrees)
//
//        SimulatedArena.getInstance().addGamePiece(previousGp)
//        x.changed().or(y.changed()).or(rotation.changed())
//            .onTrue(
//                Commands.runOnce({
//                    SimulatedArena.getInstance().removeGamePiece(previousGp)
//                    previousGp = ReefscapeCoral(
//                        Pose2d(x.get(), y.get(), Rotation2d.fromDegrees(rotation.get()))
//                    )
//                })
//            )
//    }
//
//    private fun addTunableAlgae(
//        name: String,
//        initialPose: Translation2d
//    ) {
//        var previousGp = Reefscape(initialPose)
//        val x = TunableDouble("$name/xMeters", initialPose.x)
//        val y = TunableDouble("$name/yMeters", initialPose.y)
//        val rotation = TunableDouble("$name/rotationDeg", initialPose.rotation.degrees)
//
//        SimulatedArena.getInstance().addGamePiece(previousGp)
//        x.changed().or(y.changed()).or(rotation.changed())
//            .onTrue(
//                Commands.runOnce({
//                    SimulatedArena.getInstance().removeGamePiece(previousGp)
//                    previousGp = ReefscapeCoral(
//                        Pose2d(x.get(), y.get(), Rotation2d.fromDegrees(rotation.get()))
//                    )
//                })
//            )
//    }
//
//    init {
//
//    }
//
//    // Position: 1-12, start from alliance wall
//    @JvmStatic
//    fun placeCoralOnReef(level: Int, position: Int) {
//        simArena.addGamePiece(
//            ReefscapeCoral(
//                Pose2d(2.0, 2.0, Rotation2d.fromDegrees(90.0))
//            )
//        )
//    }
//
//    fun clearCoralOnReef() {
//
//    }
//
//    fun start() {
//        // place initial algae here
//    }
//}