package com.github.kke.utils;

import EDU.gatech.cc.is.clay.NodeBoolean;
import EDU.gatech.cc.is.clay.NodeVec2;

public class v_Cache_vb extends v_Cache_v {

    private NodeBoolean embedded;

    public v_Cache_vb(NodeVec2 v, NodeBoolean b) {
        super(v);
        embedded = b;
    }

    @Override
    public void storeValue(long timestamp) {
        if (embedded.Value(timestamp)) {
            super.storeValue(timestamp);
        }
    }
}