
/*
 * NodeMsgArray.java
 */
package com.github.hy.utils;

import EDU.gatech.cc.is.clay.*;
import EDU.gatech.cc.is.communication.*;

public abstract class NodeMsgArray extends Node {
    /**
     * Return an array of Messages
     *
     * @param timestamp long, only get new information if timestamp > than last call
     *                  or timestamp == -1.
     * @return the messages recieved
     */
    public abstract Message[] Value(long timestamp);
}
