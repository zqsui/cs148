//////////////////////////////////////////////////
/////     FORWARD KINEMATICS
//////////////////////////////////////////////////

// CS148: compute and draw robot kinematics (.xform matrix for each link)
// CS148: compute and draw robot heading and lateral vectors for base movement in plane
// matrix_2Darray_to_threejs converts a 2D JavaScript array to a threejs matrix
//   for example: var tempmat = matrix_2Darray_to_threejs(link.xform);
// simpleApplyMatrix transforms a threejs object by a matrix
//   for example: simpleApplyMatrix(link.geom,tempmat);

/*
CS148: reference code has functions for:

robot_forward_kinematics
traverse_forward_kinematics_link
traverse_forward_kinematics_joint
compute_and_draw_heading
*/

function robot_forward_kinematics()
{
	mstack = [];
	stack = [];
	I = generate_identity(4);
	
	mstack.push(I);
	
	//calculate global position and orientation of robot
	T = generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2]);
	R = generate_rotation_matrix(robot.origin.rpy[0], robot.origin.rpy[1], robot.origin.rpy[2]);
	
	mstack.push(matrix_multiply(matrix_multiply(I, T), R));
	
	robot.origin.xform = mstack[mstack.length-1];
	
	stack = [];
	
	cur = robot.base;
	
	for( var i = 0 ; i < robot.links[cur].children.length ; i++)
	{
		path = [];	//record the path of the traversal
		path.push(robot.joints[robot.links[cur].children[i]].child);
		stack.push(path);
	}
		
	robot.links[cur].xform = mstack[mstack.length-1];
	compute_and_draw_heading(robot.links[cur]);
	
	//traversal both link and joint
	while ( stack.length > 0 )
	{
		path = stack.pop();
		cur = path[path.length-1];
		
		
		//get parent joint of the current link
		pjoint = robot.links[cur].parent;
		
		//calculate the corresponding translation matrix
		T = generate_translation_matrix(robot.joints[pjoint].origin.xyz[0], robot.joints[pjoint].origin.xyz[1], robot.joints[pjoint].origin.xyz[2]);
		R = generate_rotation_matrix(robot.joints[pjoint].origin.rpy[0], robot.joints[pjoint].origin.rpy[1], robot.joints[pjoint].origin.rpy[2]);
		mstack.push(matrix_multiply(matrix_multiply(mstack[mstack.length-1], T), R));
		
		//set joint transform
		robot.joints[pjoint].origin.xform = mstack[mstack.length-1];
		robot.joints[pjoint].xform = robot.joints[pjoint].origin.xform;			
		compute_and_draw_heading(robot.joints[pjoint]);
		
		//set link transform
		robot.links[cur].xform = mstack[mstack.length-1];
		compute_and_draw_heading(robot.links[cur]);
		
		if ( robot.links[cur].children == undefined )
		{
			//pop up transform by the length of the previous path
			len = path.length;
			while (len > 0)
			{
				mstack.pop();
				len--;
			}
			
		}
		else
		{
			//push tranform into stack
			for( var i = 0 ; i < robot.links[cur].children.length ; i++)
			{
				path.push(robot.joints[robot.links[cur].children[i]].child);
				stack.push(path);
			}
		}

	}
	
}

function compute_and_draw_heading(obj)
{
	tempmat = matrix_2Darray_to_threejs(obj.xform);
	simpleApplyMatrix(obj.geom,tempmat);
}