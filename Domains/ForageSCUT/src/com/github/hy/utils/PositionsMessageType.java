/*
 * PositionsMessageType.java
 */
package com.github.hy.utils;

public enum PositionsMessageType {
    HISTORY, MAP_REACHABLE, MAP_UNREACHABLE
    ,
    LEADER_REQUEST, // SourcePositionsMessage 包含若干被发现的target的坐标和一个发现者的id
    FOLLOWER_REPLY, // SourcePositionsMessage 包含一个跟随者的坐标和id
};
