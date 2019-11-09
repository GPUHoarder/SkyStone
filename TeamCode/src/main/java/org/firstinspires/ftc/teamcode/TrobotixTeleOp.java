/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp", group="Linear Opmode")

public class TrobotixTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo leftServo = null;
    private Servo rightServo = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;

    private DcMotor leftSucc = null;
    private DcMotor rightSucc = null;

    boolean servoCheck = false;
    String servoStatus = "Unlatched";

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Trobot trobot = new Trobot();
        Configure configure = new Configure(trobot, hardwareMap);
        Drive drive = new Drive(trobot);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftServo = hardwareMap.get(Servo.class, "left servo");
        rightServo = hardwareMap.get(Servo.class, "right servo");
        leftDrive  = hardwareMap.get(DcMotor.class, "front left");
        rightDrive = hardwareMap.get(DcMotor.class, "front right");
        rightDrive2 = hardwareMap.get(DcMotor.class, "rear right");
        leftDrive2 = hardwareMap.get(DcMotor.class, "rear left");

        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;

        rightSucc  = hardwareMap.get(DcMotor.class, "right intake");
        leftSucc = hardwareMap.get(DcMotor.class, "left intake");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //double drive = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;

            // leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            // rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower  = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower * 0.75);
            rightDrive.setPower(rightPower * 0.75);
            leftDrive2.setPower(leftPower * .375);
            rightDrive2.setPower(rightPower * .375);

            // Map 'a' button to block-sucking motors
            boolean buttonA = gamepad1.a;
            boolean buttonB = gamepad1.b;
            boolean buttonX = gamepad1.x;
            boolean buttonY = gamepad1.y;
            boolean DPadLeft = gamepad1.dpad_left;
            boolean DPadRight = gamepad1.dpad_right;
            if(DPadLeft) {
                leftDrive.setPower(-1);
                rightDrive.setPower(1);
                rightDrive2.setPower(-0.5);
                leftDrive2.setPower(0.5);
            } else if(DPadRight) {
                leftDrive.setPower(1);
                rightDrive.setPower(-1);
                rightDrive2.setPower(0.5);
                leftDrive2.setPower(-0.5);
            }

            if(buttonX) {
                leftServo.setPosition(0.2);
                rightServo.setPosition(0.65);
                servoCheck = true;
            }

            if (buttonY){
                leftServo.setPosition(0);
                rightServo.setPosition(1);
                servoCheck = false;
            }

            if (buttonA) {
                leftSucc.setPower(-0.5);
                rightSucc.setPower(0.5);
            } else {
                leftSucc.setPower(0);
                rightSucc.setPower(0);
            }

            if (buttonB) {
                leftSucc.setPower(0.20);
                rightSucc.setPower(-0.20);
            } else {
                leftSucc.setPower(0);
                rightSucc.setPower(0);
            }
            if (servoCheck){
                servoStatus = "Latched";
            }
            else {
                servoStatus = "Unlatched";
            }
            // Show the elapsed game time and wheel power.
            leftFront = leftDrive.getCurrentPosition();
            rightFront = rightDrive.getCurrentPosition();
            leftBack = leftDrive2.getCurrentPosition();
            rightBack = rightDrive2.getCurrentPosition();
            telemetry.addData("Left Front:", (leftFront/3));
            telemetry.addData("Right Front:", (rightFront/3));
            telemetry.addData("Left Back:", (leftBack/3));
            telemetry.addData("Right Back:", (rightBack/3));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Servos:", servoStatus);
            telemetry.update();
        }
    }
}