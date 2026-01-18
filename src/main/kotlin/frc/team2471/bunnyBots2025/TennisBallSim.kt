package frc.team2471.bunnyBots2025

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.Logger
import kotlin.math.pow

const val g = 9.80665
class TennisBallSim(val x0: Translation3d, val v0: Translation3d, val doDrag: Boolean = true) {
    var pos = x0
    var velocity = v0
    var prevVelocity = v0
    var prevAcceleration = Translation3d(0.0, 0.0, -g)
    var prevT = Timer.getFPGATimestamp()

    fun update(numSteps: Int = 1) {
        // Time from last update
        val deltaT = Timer.getFPGATimestamp() - prevT


        update(deltaT, numSteps)
    }

    fun update(deltaT: Double, numSteps: Int) {
        // time between each step
        val dt = deltaT / numSteps

        for (i in 0..<numSteps) {
            val acceleration = Translation3d(0.0, 0.0, -g) + if (doDrag) calcDragAccel(prevVelocity) else Translation3d()

            velocity += ((prevAcceleration + acceleration) / 2.0).times(dt)

            pos += ((prevVelocity + velocity) / 2.0).times(dt)

            prevAcceleration = acceleration
            prevVelocity = velocity
        }

        prevT += deltaT
    }
}

fun logFuel(name: String, vararg fuel: TennisBallSim) {
    val thing = fuel.map { it.pos }.toTypedArray()
    Logger.recordOutput(name, *thing)
}

fun MutableList<TennisBallSim>.removeFuel() {
    this.removeIf { it.pos.z < 0.0 }
}
private fun calcDragAccel(v: Translation3d): Translation3d {
    val length = v.norm
    Logger.recordOutput("VMag", length)
    val newMagnitude = calcDragAccel(length)
    Logger.recordOutput("DMag", newMagnitude)
    return v.times(-newMagnitude / length)

}
private fun calcDragAccel(s: Double): Double {
    //            rho         v^2        Cd       Frontal Area                  mass
    return (0.5 * 1.2 * s.pow(2) * 0.54 * 0.033337.pow(2) * Math.PI) / 0.057
}

/**
 * @return A pair of doubles, the first is the horizontal error from the target when the ball reaches a specific height, the second is the time error it took the ball to get there.
 */
fun calcFuelError(vx: Double, vy: Double, goalPos: Translation2d, goalTime: Double): Pair<Double, Double> {
    val fuel = TennisBallSim(Translation3d(), Translation3d(vx, 0.0, vy))
    var time = 0.0

    while (fuel.pos.z > goalPos.y || fuel.velocity.z >= 0.0) {
        fuel.update(0.02, 1)
        time += 0.02
    }

    return Pair(fuel.pos.x - goalPos.x, time - goalTime)
}