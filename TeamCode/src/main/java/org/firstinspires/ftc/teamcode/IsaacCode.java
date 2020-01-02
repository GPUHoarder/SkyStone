// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.
/**
 * Check line 141
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="IsaacAuto", group="Autonomous")

public class IsaacCode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftDrive = hardwareMap.get(DcMotor.class, "left front");
        rightDrive = hardwareMap.get(DcMotor.class, "right front");
        leftDrive2 = hardwareMap.get(DcMotor.class, "rear left");
        rightDrive2 = hardwareMap.get(DcMotor.class, "rear right");

        strafe(true, .5);
        strafe(false, .5);


    }

    private void strafe(boolean direction, double power){

        double leftPower,rightPower;
        leftPower = power;
        rightPower= power;

        if(direction){
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
            leftDrive2.setDirection(DcMotor.Direction.REVERSE);
            rightDrive2.setDirection(DcMotor.Direction.FORWARD);

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            leftDrive2.setPower(leftPower*0.5);
            rightDrive2.setPower(rightPower*0.5);

            sleep(3500);

            leftDrive.setPower(0);
            rightDrive.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);
        }
        else{
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
            leftDrive2.setDirection(DcMotor.Direction.FORWARD);
            rightDrive2.setDirection(DcMotor.Direction.REVERSE);

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            leftDrive2.setPower(leftPower*0.5);
            rightDrive2.setPower(rightPower*0.5);

            sleep(3500);

            leftDrive.setPower(0);
            rightDrive.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);
        }

    }
}
