package com.github.kke.utils;

import EDU.gatech.cc.is.clay.NodeBoolean;
import EDU.gatech.cc.is.clay.NodeVec2;
import EDU.gatech.cc.is.util.Vec2;

public class b_LessThan_v extends NodeBoolean {

    private NodeVec2 embedded = null;
    private double magnitude = 0.0;

    public b_LessThan_v(NodeVec2 v, double m) {
        embedded = v;
        magnitude = m;
    }

    @Override
    public boolean Value(long timestamp) {
        Vec2 v = embedded.Value(timestamp);
        return v.r < magnitude;
    }

}