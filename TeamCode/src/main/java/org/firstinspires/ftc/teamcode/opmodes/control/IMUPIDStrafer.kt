package org.firstinspires.ftc.teamcode.opmodes.control

import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.robot.systems.Holonomic
import kotlin.math.abs
import kotlin.math.absoluteValue

class IMUPIDStrafer (
                    var holonomic: Holonomic,
                    var imu: IMUController,
                    var strafePIDCoefficients: PIDCoefficients,
                    var angularPIDCoefficients: PIDCoefficients,
                    val strafeErrorFun: ()->Double) {

    private val angleTarget = imu.getHeading()

     var strafeErrorSum      : Double = 0.0
    private var strafeLastError     : Double = strafeErrorFun()

    private var angularErrorSum     : Double = 0.0
    private var angularLastError             = 0.0

    private var beginningSpeedLimit          = 1.0
    private var msBeforeSpeedLimit  : Long   = 0

    private var lastRuntime         : Long   = 0

    private val startingRuntime = System.currentTimeMillis()

    fun setStartingLimit(msTime: Long, limit: Double) {
        beginningSpeedLimit = limit.absoluteValue
        msBeforeSpeedLimit = msTime
    }

    fun run() {

        val strafeError = strafeErrorFun()
        val strafeOutput: Double = (
                strafePIDCoefficients.p * strafeError
              + strafePIDCoefficients.i * strafeErrorSum
              + strafePIDCoefficients.d * ((strafeError - strafeLastError) / (System.currentTimeMillis() - lastRuntime).toDouble() / 1000))
        strafeErrorSum += strafeError / ((System.currentTimeMillis() - lastRuntime).toDouble() / 1000)
        strafeLastError = strafeError


        val angularError = imu.getHeading() - angleTarget
        val angularOutput = (
                angularPIDCoefficients.p * angularError
              + angularPIDCoefficients.i * angularErrorSum
              + angularPIDCoefficients.d * angularLastError)
        angularErrorSum += angularError
        angularLastError = angularError

        holonomic.runWithoutEncoder(
                if(System.currentTimeMillis()-startingRuntime < msBeforeSpeedLimit) strafeOutput.coerceIn(-beginningSpeedLimit, beginningSpeedLimit) else strafeOutput,
                0.0,
                angularOutput)

        lastRuntime = System.currentTimeMillis()
    }

}