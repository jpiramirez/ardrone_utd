/// AR Drone driver for ROS
// Based on the JavaDrone project and rosjava.
//
// Author: Juan-Pablo Ramirez
// <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
// Summer 2012
//
// TODO: Listen to a topic to change the video channel.
// TODO: Provide the ability to send an emergency (kill) signal to the drone.



package org.ros.ardrone_utd;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.Time;

import com.codeminders.ardrone.ARDrone;
import com.codeminders.ardrone.DroneVideoListener;
import com.codeminders.ardrone.NavData;
import com.codeminders.ardrone.NavDataListener;

public class ardrone_utd extends AbstractNodeMain implements NavDataListener, DroneVideoListener {

  ARDrone drone;
  double phi, theta, gaz, psi;
  byte[] bbuf = new byte[320*240*3];
  boolean landed = true;
  double pitch, roll, yaw, vertvel;
  Time tst;

  @Override
  public GraphName getDefaultNodeName() {
    return new GraphName("ardrone_utd");
  }

  @Override
  public void navDataReceived(NavData nd) {
    phi = nd.getRoll();
    theta = nd.getPitch();
    gaz = nd.getAltitude();
    psi = nd.getYaw();
  }

  @Override 
  public void frameReceived(int startX, int startY, int w, int h, int[] rgbArray, int offset, int scansize)
  {
      for(int i=0; i < 320*240; i++)
      {
	bbuf[i*3 + 2] = (byte)(rgbArray[i] >> 16);
	bbuf[i*3 + 1] = (byte)((0xff00 & rgbArray[i]) >> 8);
	bbuf[i*3] = (byte)(0xff & rgbArray[i]);
      }
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    final Log log = connectedNode.getLog();
    Subscriber<geometry_msgs.Twist> subscriber = connectedNode.newSubscriber("cmd_vel", geometry_msgs.Twist._TYPE);
    Subscriber<std_msgs.Empty> substol = connectedNode.newSubscriber("ardrone_utd/takeoff", std_msgs.Empty._TYPE);
    Subscriber<std_msgs.Empty> subsreset = connectedNode.newSubscriber("ardrone_utd/reset", std_msgs.Empty._TYPE);
    final Publisher<geometry_msgs.Quaternion> publisher =
        connectedNode.newPublisher("ardrone_utd/navdata", geometry_msgs.Quaternion._TYPE);
    final Publisher<sensor_msgs.Image> imgpub =
        connectedNode.newPublisher("ardrone_utd/image_raw", sensor_msgs.Image._TYPE);
        
	try{
		drone = new ARDrone();
		drone.connect();
		drone.clearEmergencySignal();
		drone.waitForReady(3000);
		drone.trim();
		drone.addNavDataListener(this);
		drone.addImageListener(this);
	}
	catch(Throwable e)
	{
		e.printStackTrace();
	}


    substol.addMessageListener(new MessageListener<std_msgs.Empty>() {
      @Override
      public void onNewMessage(std_msgs.Empty message) {
          try
          {
            if(landed)
            {
	      drone.takeOff();
	      log.info("The drone is taking off.");
            }
            else
            {
              drone.land();
              log.info("The drone is landing.");
            }
            landed = !landed;
          }
          catch (Throwable e)
          {
            e.printStackTrace();
          }
      }
    });
    
    subsreset.addMessageListener(new MessageListener<std_msgs.Empty>() {
      @Override
      public void onNewMessage(std_msgs.Empty message) {
          try
          {
	      drone.clearEmergencySignal();
	      log.info("Trying to clear the emergency state.");
          }
          catch (Throwable e)
          {
            e.printStackTrace();
          }
      }
    });
    
    // The Twist message is interpreted just as in Brown's ardrone_brown package.
    subscriber.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
      @Override
      public void onNewMessage(geometry_msgs.Twist message) {
        geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
        val = message.getLinear();
        roll = val.getY();
        pitch = val.getX();
        vertvel = val.getZ();
        val = message.getAngular();
        yaw = val.getZ();
	try
	{
	  drone.move((float)roll, (float)pitch, (float)vertvel, (float)yaw);
	}
	catch (Throwable e)
	{
	  e.printStackTrace();
	}
        log.debug("Drone commands: " + roll + " " + pitch + " " + vertvel + " " + yaw);        
      }
    });
  
    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;

      @Override
      protected void setup() {
        sequenceNumber = 0;
      }

      @Override
      protected void loop() throws InterruptedException {
        std_msgs.Header imghead = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);

        geometry_msgs.Quaternion str = publisher.newMessage();
        sensor_msgs.Image imagemess = imgpub.newMessage();

	imghead.setSeq(sequenceNumber);
	imghead.setStamp(tst.fromMillis(System.currentTimeMillis()));
	imghead.setFrameId("0");

        imagemess.setData(bbuf);
        imagemess.setEncoding("8UC3");
        imagemess.setWidth(320);
        imagemess.setHeight(240);
        imagemess.setStep(0);
        imagemess.setIsBigendian((byte)0);
        imagemess.setHeader(imghead);

	str.setX(phi);
	str.setY(theta);
	str.setZ(gaz);
	str.setW(psi);
        publisher.publish(str);
	imgpub.publish(imagemess);
        sequenceNumber++;
        Thread.sleep(100);
      }
    });  


  }

}
