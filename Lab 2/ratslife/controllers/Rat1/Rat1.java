// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.LightSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

import java.util.Random;

public class Rat1 extends Robot {

  protected final int timeStep = 32;
  protected final double maxSpeed = 300;
  protected final double[] collisionAvoidanceWeights = {0.06,0.03,0.015,0.0,0.0,-0.015,-0.03,-0.06};
  protected final double[] slowMotionWeights = {0.0125,0.00625,0.0,0.0,0.0,0.0,0.00625,0.0125};

  protected Accelerometer accelerometer;
  protected Motor leftMotor, rightMotor;
  protected DistanceSensor[] distanceSensors = new DistanceSensor[8];
  protected LightSensor[] lightSensors = new LightSensor[8];

  public Rat1() {
    accelerometer = getAccelerometer("accelerometer");
    leftMotor = getMotor("left wheel motor");
    rightMotor = getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);
    
    for (int i=0;i<8;i++) {
      distanceSensors[i] = getDistanceSensor("ps"+i);
      distanceSensors[i].enable(timeStep);
      lightSensors[i] = getLightSensor("ls"+i);
      lightSensors[i].enable(timeStep);
    }
    batterySensorEnable(timeStep);
  }

  public void run() {

    Random r = new Random();
    boolean turn = false;
    boolean right = false;
    boolean seeFeeder = false;
    double battery;
    double oldBattery = -1.0;
    int image[];
    double distance[] = new double[8];
    double leftSpeed, rightSpeed;

    while (step(timeStep) != -1) {

      // read sensor information
      image = camera.getImage();
      for(int i=0;i<8;i++) distance[i] = distanceSensors[i].getValue();
      battery = batterySensorGetValue();

      // obstacle avoidance behavior
      leftSpeed  = maxSpeed;
      rightSpeed = maxSpeed;
      //for (int i=0;i<8;i++) {
        //leftSpeed  -= (slowMotionWeights[i]+collisionAvoidanceWeights[i])*distance[i];
        //rightSpeed -= (slowMotionWeights[i]-collisionAvoidanceWeights[i])*distance[i];
      //}
      /// To detect an obstacle use sensors 3,4
      if(distance[3] > 500 || distance[4] > 500) {
        leftSpeed = maxSpeed;  
        rightSpeed = -maxSpeed; // Turn Right.
      }
      else { 
        //The distance from the left hand side wall is okay, i obey to the LWF rule continue straight
        if(distance[5] > 300) {
          leftSpeed = maxSpeed;
          rightSpeed = maxSpeed;
        }
        else {
          leftSpeed = maxSpeed/8;
          rightSpeed = maxSpeed;
        }
        
        //I had an 180 left turn, let me fix my position parallel to the wall to avoid crashing to it
        if(distance[5] > 300){
          //System.out.println("Oh I came too close");
          leftSpeed = maxSpeed/2.5;
          rightSpeed = maxSpeed;
        }
      }
      
      
     
      //recharging behavior
      if (battery > oldBattery) {
        leftSpeed  = 0.0;
        rightSpeed = 0.0;
      }
      oldBattery = battery;
      
      // Added negative(-) to make the robot work backwards.
      leftMotor.setVelocity(-0.00628 * leftSpeed);
      rightMotor.setVelocity(-0.00628 * rightSpeed);
    }
    // Enter here exit cleanup code
  }

  public static void main(String[] args) {
    Rat1 rat1 = new Rat1();
    rat1.run();
  }
}
