/*
Copyright (c) 2016 Robert Atkinson
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This example is designed to show how to identify a target, get the robot's position, and then plan
 * and execute an approach path to the target.
 *
 * This OpMode uses two "utility" classes to abstract (hide) the hardware and navigation GUTs.
 * These are:  Robot_OmniDrive and Robot_Navigation.
 *
 * This LinearOpMode uses basic hardware and nav calls to drive the robot in either manual or auto mode.
 * AutoMode is engaged by pressing and holding the Left Bumper.  Release the Bumper to return to Manual Mode.
 *
 *  *ManualMode* simply uses the joysticks to move the robot in three degrees of freedom.
 *  - Left stick X (translate left and right)
 *  - Left Stick Y (translate forward and backwards)
 *  - Right Stick X (rotate CW and CCW)
 *
 *  *AutoMode* will approach the image target and attempt to reach a position directly in front
 *  of the center line of the image, with a predefined stand-off distance.
 *
 *  To simplify this example, a gyro is NOT used.  Therefore there is no attempt being made to stabilize
 *  strafing motions, or to perform field-centric driving.
 *
 */

@Autonomous(name="Blue Side: Skystone", group="Autonomous")
public class BlueSkystone extends LinearOpMode {

    private double counter       =  0;
    private double timeoutFirst  =  0;
    private double timeoutSecond =  0;
    final double TARGET_DISTANCE =  225.0;    // Hold robot's center 400 mm from target

    /* Declare OpMode members. */
    Robot_OmniDrive     robot    = new Robot_OmniDrive();   // Use Omni-Directional drive system
    Robot_Navigation    nav      = new Robot_Navigation();  // Use Image Tracking library

    final double SPEED           =  0.06;

    @Override
    public void runOpMode() {

        // Initialize the robot and navigation
        robot.initDrive(this);
        nav.initVuforia(this, robot);

        // Activate Vuforia (this takes a few seconds)
        nav.activateTracking();

        // Wait for the game to start (driver presses PLAY)
        /*while (!isStarted()) {
            // Prompt User
            telemetry.addData(">", "Press start");

            // Display any Nav Targets while we wait for the match to start
            nav.targetsAreVisible();
            nav.addNavTelemetry();
            telemetry.update();
        }*/
        waitForStart();
        // run until the end of the match (driver presses STOP)
        robot.encoderDrive(SPEED,0.78, 0.78, -0.78, -0.78, 1.45);
        robot.stopMotor();
        sleep(2050);
        while (opModeIsActive()) {

            telemetry.addData(">", "Press Left Bumper to track target");

            // auto drive or manual drive?
            // In auto drive, the robot will approach any target it can see and then press against it
            // In manual drive the robot responds to the Joystick.

            if (nav.targetsAreVisible() && (nav.getRobotY() < 75 && nav.getRobotY() > -35)) {
                robot.encoderDrive(SPEED,10, 10, -10, -10, 1.2);
                robot.setServo(0);
                robot.encoderDrive(0, 0, 0, 0, 0, 2.0);

                robot.encoderDrive(SPEED,-10, -10, 10, 10, 1.3);
                telemetry.addLine("First");
                telemetry.update();
                robot.encoderDrive(SPEED, 56, -56, 56, -56, 2.4 + counter);

                robot.encoderDrive(0, 0, 0, 0, 0, 2.0);


//                robot.encoderDrive(SPEED,10, 10, -10, -10, 1.2);
//                robot.setServo(0.5);
//                robot.encoderDrive(SPEED,-10, -10, 10, 10, 1.2);
//
//                robot.encoderDrive(SPEED, -80, 80, -80, 80, timeoutSecond);
//
//                robot.encoderDrive(0.2,10, 10, -10, -10, 1.2);
//                robot.setServo(0);
//                robot.encoderDrive(0.2,-10, -10, 10, 10, 1.2);
//
//                robot.encoderDrive(0.2, 80, -80, 80, -80, timeoutSecond);
//
//                robot.encoderDrive(SPEED,10, 10, -10, -10, 1.1);
//                robot.setServo(0.5);
                //robot.encoderDrive(SPEED,-10, -10, 10, 10, 1.1);
                telemetry.addLine("Second");
                telemetry.update();
                robot.encoderDrive(SPEED, -10, 10, -10, 10, 1.5);
                telemetry.addLine("Third");
                telemetry.update();
                robot.stopMotor();
                sleep(30000);

            } else {
                // Drive the robot using the joysticks
                nav.addNavTelemetry();
                robot.driveMotor(SPEED);
                sleep(560);
                robot.stopMotor();
                sleep(3000);
                counter += 0.2;
                timeoutFirst = counter + 1.8;
                timeoutSecond = counter + 2;
                telemetry.update();

            }

            // Build telemetry messages with Navigation Information;
            nav.addNavTelemetry();

            //  Move the robot according to the pre-determined axis motions
//            if (!robot.checkAlignment())  {
//                robot.align();
//            }
//            robot.moveRobot();
            telemetry.update();
        }

        telemetry.addData(">", "Shutting Down. Bye!");
        telemetry.update();
    }
}