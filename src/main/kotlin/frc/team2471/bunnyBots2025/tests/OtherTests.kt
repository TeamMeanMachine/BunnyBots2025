package frc.team2471.bunnyBots2025.tests

import com.ctre.phoenix6.controls.DutyCycleOut
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team2471.bunnyBots2025.OI
import frc.team2471.bunnyBots2025.Shooter
import frc.team2471.bunnyBots2025.Shooter.getAngleAndRPS
import frc.team2471.bunnyBots2025.Turret
import org.team2471.frc.lib.control.commands.finallyWait
import org.team2471.frc.lib.control.commands.finallyWaitUntil
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnce
import org.team2471.frc.lib.control.commands.runOnceCommand
import org.team2471.frc.lib.control.commands.sequenceCommand
import org.team2471.frc.lib.control.dPad
import org.team2471.frc.lib.ctre.setCANCoderAngle
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.asDegreesPerSecond
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.rotations
import kotlin.math.absoluteValue


fun turretPercentOutputTest(): Command = runCommand {
//    Turret.turretMotor.setControl(DutyCycleOut(OI.driverController))
}
fun zeroPivot(): Command = runOnceCommand {
    println("Zeroing Pivot angle: ${Turret.pivotEncoderAngle.asDegrees}")
    Turret.pivotEncoderOffset = 18.0 - Turret.pivotEncoder.absolutePosition.valueAsDouble.rotations.asDegrees
    println("Zeroing Pivot angle: ${Turret.pivotEncoderAngle.asDegrees}")
    Turret.pivotMotor.setPosition(Turret.pivotEncoderAngle)

}

fun zeroTurret(): Command = runOnceCommand {
    println("Zeroing turret. before angle: ${Turret.turretEncoderAngle}")
    Turret.lampreyEncoderOffset = Turret.rawLampreyAngle.asDegrees
    println("Zeroing turret. after angle: ${Turret.pivotEncoderAngle.asDegrees}")
    Turret.turretMotor.setPosition(Turret.turretEncoderAngle)
}

fun shooterSysIDRight(): Command = sequenceCommand(
    Shooter.sysIDRightRoutine.quasistatic(SysIdRoutine.Direction.kForward).finallyWaitUntil {
        Shooter.motorSpeedRight.asDegreesPerSecond.absoluteValue < 1.0
    }.finallyWait(1.0),
            Shooter.sysIDRightRoutine.quasistatic(SysIdRoutine.Direction.kReverse).finallyWaitUntil {
                Shooter.motorSpeedRight.asDegreesPerSecond.absoluteValue < 1.0
            }.finallyWait(1.0),
    Shooter.sysIDRightRoutine.dynamic(SysIdRoutine.Direction.kForward).finallyWaitUntil {
        Shooter.motorSpeedRight.asDegreesPerSecond.absoluteValue < 1.0
    }.finallyWait(1.0),
    Shooter.sysIDRightRoutine.dynamic(SysIdRoutine.Direction.kReverse)
)

fun shooterSysIDLeft(): Command = sequenceCommand(
    Shooter.sysIDLeftRoutine.quasistatic(SysIdRoutine.Direction.kForward).finallyWaitUntil {
        Shooter.motorSpeedLeft.asDegreesPerSecond.absoluteValue < 1.0
    }.finallyWait(1.0),
    Shooter.sysIDLeftRoutine.quasistatic(SysIdRoutine.Direction.kReverse).finallyWaitUntil {
        Shooter.motorSpeedLeft.asDegreesPerSecond.absoluteValue < 1.0
    }.finallyWait(1.0),
    Shooter.sysIDLeftRoutine.dynamic(SysIdRoutine.Direction.kForward).finallyWaitUntil {
        Shooter.motorSpeedLeft.asDegreesPerSecond.absoluteValue < 1.0
    }.finallyWait(1.0),
    Shooter.sysIDLeftRoutine.dynamic(SysIdRoutine.Direction.kReverse)
)


fun printShooterCurves(): Command = runOnce {
    var angles = mutableMapOf<Double, Double>()
    var speeds = mutableMapOf<Double, Double>()

    for (i in 4..10) {
        val dist = (i.toDouble() / 2.0)
        val angleAndSpeed = getAngleAndRPS(dist)

        angles[dist] = angleAndSpeed.first
        speeds[dist] = angleAndSpeed.second
    }

    println("Angle curve")
    angles.forEach { dist, angle ->
        println("put($dist, $angle)")
    }

    println("Speed curve")
    speeds.forEach { dist, speed ->
        println("put($dist, $speed)")
    }
}

fun printLampreyCurve(): Command = runCommand(Turret) {
    println("Lamprey, Motor")
    val increment = 5
    for (i in 0..360/increment) {
        val angle = (i * increment).toDouble()
        Turret.turretSetpoint = angle.degrees

        while (Turret.turretSetpointError > 0.5.degrees) {}

        println("${Turret.unCorrectedLampreyAngle}, ${Turret.turretMotorAngle}")
    }
}