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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="TeleOp Final Robit", group="Linear Opmode")

public class TrobotixTeleOpFinal extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;
    private DcMotor elevatorMotor = null;
    private DcMotor clawMotor = null;

    private Servo autoServo = null;

    static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    =  2.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 0.2 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.2;

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        rightDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightDrive2 = hardwareMap.get(DcMotor.class, "rear_right");
        leftDrive2 = hardwareMap.get(DcMotor.class, "rear_left");
        autoServo = hardwareMap.get(Servo.class, "auto_servo");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator");
        clawMotor = hardwareMap.get(DcMotor.class, "claw");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Path0",  "Starting at %7d",
                elevatorMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            double leftPower2;
            double rightPower2;

            leftPower = -gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;

            leftPower2 = -gamepad2.left_stick_y;
            rightPower2 = gamepad2.left_stick_y;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower * 0.75);
            rightDrive.setPower(rightPower * 0.75);
            leftDrive2.setPower(leftPower * 0.75);
            rightDrive2.setPower(rightPower * 0.75);

            // Map 'a' button to block-sucking motors


            boolean DPadLeft = gamepad1.dpad_left;
            boolean DPadRight = gamepad1.dpad_right;

            boolean bumperL = gamepad1.left_bumper;
            boolean bumperR = gamepad1.right_bumper;

            boolean buttonA = gamepad1.a;
            boolean buttonB = gamepad1.b;
            boolean buttonX = gamepad1.x;
            boolean buttonY = gamepad1.y;




            //strafe
            if (DPadLeft) {
                leftDrive.setPower(-0.5);
                rightDrive.setPower(-0.5);
                rightDrive2.setPower(0.5);
                leftDrive2.setPower(0.5);
            } else if (DPadRight) {
                leftDrive.setPower(0.5);
                rightDrive.setPower(0.5);
                rightDrive2.setPower(-0.5);
                leftDrive2.setPower(-0.5);
            }




            //servos
            if (bumperR) {
                autoServo.setPosition(0.5);     //right is up
            }
            if (bumperL) {
                autoServo.setPosition(0);       //left is down
            }




            //elevator claw
            if (buttonA) {
                clawMotor.setPower(1);
            }
            else{
                clawMotor.setPower(0);
            }
            if (buttonB){
                clawMotor.setPower(-1);
            }
            else{
                clawMotor.setPower(0);
            }




            //elevator lift
            if (buttonX){
                elevatorMotor.setPower(1);
            }
            else{
                elevatorMotor.setPower(0);
            }
            if (buttonY){
                elevatorMotor.setPower(-1);
            }
            else{
                elevatorMotor.setPower(0);
            }





            // Show the elapsed game time and wheel power.

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

        }
    }
    public void encoderDrive(double speed,
                             double elevatorInches){
        int newElevatorTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newElevatorTarget = elevatorMotor.getCurrentPosition() + (int)(elevatorInches * COUNTS_PER_INCH);
            elevatorMotor.setTargetPosition(newElevatorTarget);

            // Turn On RUN_TO_POSITION
            elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            elevatorMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (elevatorMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", newElevatorTarget);
                telemetry.addData("Path2",  "Running at %7d", elevatorMotor.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            elevatorMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
