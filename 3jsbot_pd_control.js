//////////////////////////////////////////////////
/////     MOTION CONTROL ROUTINES 
//////////////////////////////////////////////////

// CS148: add PD controller here
function robot_pd_control () {
	var curdate = new Date();
	
	/* robot.joints[active_joint].servo.p_desired = curdate.getSeconds()/60*2*Math.PI;
	error = robot.joints[active_joint].servo.p_desired - robot.joints[active_joint].angle;
    robot.joints[active_joint].control += error * robot.joints[x].servo.gains;*/
	
	for(x in robot.joints){
		robot.joints[x].servo.p_desired = curdate.getSeconds()/60*2*Math.PI;
		error = robot.joints[x].servo.p_desired - robot.joints[active_joint].angle;
		robot.joints[x].control += error * robot.joints[x].servo.gains;
    }
}

