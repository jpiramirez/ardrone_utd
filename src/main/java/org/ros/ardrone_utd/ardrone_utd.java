/// AR Drone driver for ROS
// Based on the JavaDrone project and rosjava.
//
// Author: Juan-Pablo Ramirez
// <pablo.ramirez@utdallas.edu>
// The University of Texas at Dallas
//


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
import com.codeminders.ardrone.ARDrone.VideoChannel;

public class ardrone_utd extends AbstractNodeMain implements NavDataListener, DroneVideoListener {

ARDrone drone;
double phi, theta, gaz, psi;
byte[] bbuf = new byte[320*240*3];
boolean landed = true;
boolean videohoriz = true;
boolean emergency = false;
double pitch, roll, yaw, vertvel;
Time tst;
int imwidth, imheight;

@Override
public GraphName getDefaultNodeName() {
	return new GraphName("ardrone");
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
	for(int i=0; i < imwidth*imheight; i++)
	{
		bbuf[i*3 + 2] = (byte)((rgbArray[i] >> 16) & 0xff);
		bbuf[i*3 + 1] = (byte)((rgbArray[i] >> 8) & 0xff);
		bbuf[i*3] = (byte)(0xff & rgbArray[i]);
	}
}

@Override
public void onStart(final ConnectedNode connectedNode) {
	final Log log = connectedNode.getLog();
	Subscriber<geometry_msgs.Twist> subscriber = connectedNode.newSubscriber("cmd_vel", geometry_msgs.Twist._TYPE);
	Subscriber<std_msgs.Empty> substol = connectedNode.newSubscriber("ardrone/takeoff", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subsreset = connectedNode.newSubscriber("ardrone/reset", std_msgs.Empty._TYPE);
	Subscriber<std_msgs.Empty> subschannel = connectedNode.newSubscriber("ardrone/zap", std_msgs.Empty._TYPE);
	final Publisher<geometry_msgs.Quaternion> publisher =
		connectedNode.newPublisher("ardrone/navdata", geometry_msgs.Quaternion._TYPE);
	final Publisher<sensor_msgs.Image> imgpub =
		connectedNode.newPublisher("ardrone/image_raw", sensor_msgs.Image._TYPE);
	final Publisher<sensor_msgs.CameraInfo> caminfopub =
		connectedNode.newPublisher("ardrone/camera_info", sensor_msgs.CameraInfo._TYPE);
		
	try{
		drone = new ARDrone();
		drone.connect();
		drone.clearEmergencySignal();
		drone.waitForReady(3000);
		drone.trim();
		drone.addNavDataListener(this);
		drone.addImageListener(this);
		drone.selectVideoChannel(VideoChannel.HORIZONTAL_ONLY);
		imwidth = 320;
		imheight = 240;
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
			if(emergency)
			{
				drone.clearEmergencySignal();
				log.info("Trying to clear the emergency state.");
			}
			else
			{
				drone.sendEmergencySignal();
				log.info("Sending a kill signal to the drone.");
			}
				emergency = !emergency;
			}
		catch (Throwable e)
		{
			e.printStackTrace();
		}
	}
	});

	subschannel.addMessageListener(new MessageListener<std_msgs.Empty>() {
	@Override
	public void onNewMessage(std_msgs.Empty message) {
		try
		{
			if(videohoriz)
			{
				drone.selectVideoChannel(VideoChannel.VERTICAL_ONLY);
				imwidth = 176;
				imheight = 144;
				log.info("Attempting to use the vertical camera.");
			}
			else
			{
				drone.selectVideoChannel(VideoChannel.HORIZONTAL_ONLY);
				imwidth = 320;
				imheight = 240;
				log.info("Attempting to use the horizontal camera.");
			}
			videohoriz = !videohoriz;
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
			drone.move(-(float)roll, -(float)pitch, (float)vertvel, -(float)yaw);
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
			sensor_msgs.CameraInfo caminfomsg = caminfopub.newMessage();

			//So far I've been unable to figure out how to fill the K and P matrices
			//using rosjava --J.Pablo
			//double[] K = {imwidth/2.0, 0, imwidth/2.0, 0, 160, 120, 0, 0, 1};
			//double[] P = {160, 0, 160, 0, 0, 160, 120, 0, 0, 0, 1, 0};

			imghead.setSeq(sequenceNumber);
			tst = connectedNode.getCurrentTime();
			imghead.setStamp(tst);
			imghead.setFrameId("0");

			imagemess.setData(bbuf);
			imagemess.setEncoding("8UC3");
			imagemess.setWidth(imwidth);
			imagemess.setHeight(imheight);
			imagemess.setStep(imwidth*3);
			imagemess.setIsBigendian((byte)0);
			imagemess.setHeader(imghead);

			caminfomsg.setHeader(imghead);
			caminfomsg.setWidth(imwidth);
			caminfomsg.setHeight(imheight);
			caminfomsg.setDistortionModel("plumb_bob");
			//caminfomsg.setK(K);
			//caminfomsg.setP(P);

			str.setX(phi);
			str.setY(theta);
			str.setZ(gaz);
			str.setW(psi);
			publisher.publish(str);
			imgpub.publish(imagemess);
			caminfopub.publish(caminfomsg);
			sequenceNumber++;
			Thread.sleep(50);
		}
	});  


}

}
