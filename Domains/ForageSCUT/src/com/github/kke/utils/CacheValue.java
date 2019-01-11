package com.github.kke.utils;

import EDU.gatech.cc.is.util.Vec2;

public interface CacheValue {
    public void storeValue(long timestamp);
    public Vec2 retreiveValue();
}