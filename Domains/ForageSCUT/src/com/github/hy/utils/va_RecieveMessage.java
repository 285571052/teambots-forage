
/*
 * va_RecieveMessage.java
 */
package com.github.hy.utils;

import EDU.gatech.cc.is.communication.Transceiver;
import java.util.Enumeration;
import java.util.LinkedList;
import java.util.List;
import EDU.gatech.cc.is.communication.*;

public class va_RecieveMessage extends NodeMsgArray {
    /**
     * Turn debug printing on or off.
     */
    public static final boolean DEBUG = false;
    private Transceiver abstract_robot;
    private Enumeration messagesin; // to buffer incoming messages
    long lasttime = 0;
    Message[] last_val = new Message[0];

    /**
     * Instantiate a va_RecieveMessage node.
     *
     * @param ar VisualObjectSensor, abstract_robot object that provides hardware
     *           support.
     */
    public va_RecieveMessage(Transceiver ar) {
        if (DEBUG)
            System.out.println("va_RecieveMessage: instantiated");
        abstract_robot = ar;

        /*--- Instantiate the message buffer ----*/
        messagesin = abstract_robot.getReceiveChannel();// COMMUNICATION
    }

    /**
     * Return an array of Message from other teammates.
     *
     * @param timestamp long, only get new information if timestamp > than last call
     *                  or timestamp == -1.
     * @return the messages recieved
     */
    public Message[] Value(long timestamp) {
        if ((timestamp > lasttime) || (timestamp == -1)) {
            /*--- reset the timestamp ---*/
            if (timestamp > 0)
                lasttime = timestamp;

            /*--- while there are messages in the queue ---*/
            List<Message> retList = new LinkedList<Message>();
            while (abstract_robot.connected() && messagesin.hasMoreElements()) {
                Message msg = (Message) messagesin.nextElement();
                retList.add(msg);
            }

            last_val = new Message[retList.size()];
            last_val = retList.toArray(last_val);

        }

        Message[] retval = new Message[last_val.length];
        for (int i = 0; i < last_val.length; i++) {
            try {
                retval[i] = (Message) last_val[i].clone();
            } catch (CloneNotSupportedException e) {
            }
        }

        return (retval);
    }
}
