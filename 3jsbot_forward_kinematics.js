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


	var heading_local = [[1],[0],[0],[1]];
	//var heading_local = [1,0,0,1];
	//console.log(heading_local);
	robot_heading = matrix_multiply(robot.origin.xform, heading_local);

	//console.log(robot_heading);
	var heading_mat = matrix_2Darray_to_threejs(vectortomatrix(robot_heading));

	var lateral_local = [[0],[0],[1],[1]];
	//console.log(heading_local);
	robot_lateral = matrix_multiply(robot.origin.xform, lateral_local);
	//console.log(robot_heading);
	var lateral_mat = matrix_2Darray_to_threejs(vectortomatrix(robot_lateral));

	 if (typeof heading_geom === 'undefined') {
            var temp_geom = new THREE.CubeGeometry(0.3, 0.3, 0.3);
            var temp_material = new THREE.MeshBasicMaterial( {color: 0x00ffff} )
            heading_geom = new THREE.Mesh(temp_geom, temp_material);
            scene.add(heading_geom);
        }
        if (typeof lateral_geom === 'undefined') {
            var temp_geom = new THREE.CubeGeometry(0.3, 0.3, 0.3);
            var temp_material = new THREE.MeshBasicMaterial( {color: 0x008888} )
            lateral_geom = new THREE.Mesh(temp_geom, temp_material); 
            scene.add(lateral_geom);
        }


	simpleApplyMatrix(heading_geom, heading_mat);
	simpleApplyMatrix(lateral_geom, lateral_mat);
	
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
		quaternion = quaternion_from_axisangle(robot.joints[pjoint].axis, robot.joints[pjoint].angle);
		R_axis = quaternion_to_rotation_matrix(quaternion);
		mstack.push(matrix_multiply(matrix_multiply(matrix_multiply(mstack[mstack.length-1], T), R),R_axis));
		
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