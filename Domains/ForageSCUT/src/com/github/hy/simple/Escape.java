
/*
 * escape.java
 */
package com.github.hy.simple;

import EDU.gatech.cc.is.util.Vec2;
import EDU.gatech.cc.is.abstractrobot.*;
import EDU.gatech.cc.is.clay.*;

public class Escape extends ControlSystemMFN150 {
	public final static boolean DEBUG = true;
	private NodeVec2 turret_configuration;
	private NodeVec2 steering_configuration;
	private i_FSA_ba STATE_MACHINE;

	/**
	 * Configure the control system using Clay.
	 */
	public void configure() {
		// ======
		// Set some initial hardware configurations.
		// ======
		abstract_robot.setObstacleMaxRange(3.0); // don't consider
		// things further away
		abstract_robot.setBaseSpeed(abstract_robot.MAX_TRANSLATION);

		// --- obstacles
		NodeVec2Array // the sonar readings
		PS_OBS = new va_Obstacles_r(abstract_robot); // 获取感知到障碍物的位置

		// ======
		// motor schemas
		// ======
		// avoid obstacles
		NodeVec2 MS_AVOID_OBSTACLES = new v_Avoid_va(1.5, abstract_robot.RADIUS + 0.1, PS_OBS);
		// 每个障碍物会产生一个排斥量来促使机器人调整移动方向, 该函数返回最后综合的调整

		// noise vector
		NodeVec2 MS_NOISE_VECTOR = new v_Noise_(5, seed);
		// 在每个状态的每个动作时, 都增加一定的随机变化量

		// swirl obstacles wrt noise
		NodeVec2 MS_SWIRL_OBSTACLES_NOISE = new v_Swirl_vav(2.0, abstract_robot.RADIUS + 0.1, PS_OBS, MS_NOISE_VECTOR);

		// ======
		// AS_WANDER
		// ======
		v_StaticWeightedSum_va AS_WANDER = new v_StaticWeightedSum_va();

		AS_WANDER.weights[0] = 1.0;
		AS_WANDER.embedded[0] = MS_AVOID_OBSTACLES; // 避开障碍

		AS_WANDER.weights[1] = 1.0;
		AS_WANDER.embedded[1] = MS_NOISE_VECTOR; // 随机移动

		AS_WANDER.weights[2] = 1.0;
		AS_WANDER.embedded[2] = MS_SWIRL_OBSTACLES_NOISE; // 随机旋转

		// ======
		// STATE_MACHINE
		// ======
		STATE_MACHINE = new i_FSA_ba();

		STATE_MACHINE.state = 0;

		// ======
		// STEERING
		// ======
		v_Select_vai STEERING = new v_Select_vai((NodeInt) STATE_MACHINE);

		STEERING.embedded[0] = AS_WANDER;

		// ======
		// TURRET
		// ======
		v_Select_vai TURRET = new v_Select_vai((NodeInt) STATE_MACHINE);

		TURRET.embedded[0] = AS_WANDER;

		turret_configuration = TURRET;
		steering_configuration = STEERING;
	}

	/**
	 * Called every timestep to allow the control system to run.
	 */
	public int takeStep() {
		Vec2 result;
		long curr_time = abstract_robot.getTime();

		// STEER
		result = steering_configuration.Value(curr_time);
		abstract_robot.setSteerHeading(curr_time, result.t);
		abstract_robot.setSpeed(curr_time, result.r);

		// TURRET
		result = turret_configuration.Value(curr_time);
		abstract_robot.setTurretHeading(curr_time, result.t);

		// STATE DISPLAY
		int state = STATE_MACHINE.Value(curr_time);
		if (state == 0)
			abstract_robot.setDisplayString("wander");

		return (CSSTAT_OK);
	}
}
