
/*
 * PositionsMessage.java
 */
package com.github.hy.utils;

import java.io.*;
import EDU.gatech.cc.is.util.*;
import EDU.gatech.cc.is.communication.Message;

public class PositionsMessage extends Message implements Cloneable, Serializable {
    /**
     * the positions from the sender.
     */
    public Vec2[] val = new Vec2[0];

    /**
     * create a position message.
     *
     * @param p Vec2[], positions.
     */
    public PositionsMessage(Vec2[] p) {
        val = p;
    }
}
