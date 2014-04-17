//////////////////////////////////////////////////
/////     INVERSE KINEMATICS 
/////     Resolved-rate IK with geometric jacobian
//////////////////////////////////////////////////

// CS148: generate joint controls to move robot to move robot endeffector to target location

/*
CS148: reference code has functions for:

robot_inverse_kinematics
iterate_inverse_kinematics
*/



function robot_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {
    // compute joint angle controls to move location on specified link to Cartesian location
    if (update_ik) {
        iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos)
        endeffector_geom.visible = true;
        target_geom.visible = true;

        var endeffector_pos = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_local_pos)
        var endeffector_mat = matrix_2Darray_to_threejs(vectortomatrix(endeffector_pos));
        simpleApplyMatrix(endeffector_geom, endeffector_mat);

        var target_mat = matrix_2Darray_to_threejs(vectortomatrix(target_pos));
		simpleApplyMatrix(target_geom, target_mat);

		//simpleApplyMatrix(target_geom, );
    }
    else {
        endeffector_geom.visible = false;
        target_geom.visible = false;
    }
    update_ik = false;



}

function iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {

    var endeffector_global_pos = matrixtovector(matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_local_pos));
    //console.log(matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_local_pos));
    var dis_vec = vector_substraction(endeffector_global_pos, target_pos);
    //console.log("end",endeffector_global_pos[0], endeffector_global_pos[1], endeffector_global_pos[2])
    //console.log("dis", dis_vec[0], dis_vec[1], dis_vec[2]);
    
    //if ( Math.abs(dis_vec[0]) > 0.0005 || Math.abs(dis_vec[1]) > 0.0005 || Math.abs(dis_vec[2]) > 0.0005 ) 
    //{

        //building jacobian matrix
        
        var cur_joint = endeffector_joint;
        var j = [];
        while ( cur_joint != "base" )
        {
            //console.log(cur_joint);
            //doing some calculation here
            var cur_joint_origin_global_pos = matrixtovector(matrix_multiply(robot.joints[cur_joint].xform, [[0],[0],[0],[1]]));//postomatrix(robot.joints[cur_joint].origin.xyz)));
            //console.log("---------");
            //console.log(matrix_multiply(robot.joints[cur_joint].xform, [[0],[0],[0],[1]]));
            
            
            //console.log(robot.joints[cur_joint].xform);
            //var R = generate_rotation_matrix(robot.joints[cur_joint].origin.rpy[0], robot.joints[cur_joint].origin.rpy[1], robot.joints[cur_joint].origin.rpy[2]);
            var R = clean_rotation_matrix(robot.joints[cur_joint].xform);
            //var R = robot.joints[pjoint].xform;
            var z = matrixtovector(matrix_multiply(R, [[robot.joints[cur_joint].axis[0]],[robot.joints[cur_joint].axis[1]],[robot.joints[cur_joint].axis[2]],[1]]));

            var jv = vector_cross(z, vector_substraction(cur_joint_origin_global_pos, endeffector_global_pos));
            var jw = z;
            
            //console.log([jv[0], jv[1], jv[2], jw[0], jw[1], jw[2]]);
            j.push([jv[0], jv[1], jv[2], jw[0], jw[1], jw[2]]);

            

            
            var plink = robot.joints[cur_joint].parent;
            if ( plink == "base")
                {
                    cur_joint = "base";
                    //cur_joint_origin_global_pos = matrixtovector1(matrix_multiply(robot.joints[cur_joint].xform, postomatrix(robot.joints[cur_joint].origin.xyz)));

                }
            else cur_joint = robot.links[plink].parent;
        }

        //console.log(j);

        j = matrix_transpose(j);

        //console.log(j);

        //jacobian transpose


        dis_vec = vector_substraction(endeffector_global_pos, target_pos);
        //dis_vec = vector_substraction(target_pos, endeffector_global_pos);
        var dx = [[dis_vec[0]], [dis_vec[1]],[dis_vec[2]], [0], [0], [0]];
        var alpha = 0.1;

        j_t = matrix_transpose(j);

        
        //pseudo inverse
        d_theta = matrix_multiply(matrix_multiply(numeric.inv(matrix_multiply(j_t, j)), j_t), dx);


        //jacobian transpose 
        /* 
        d_theta = matrix_multiply(j_t, dx);
        alpha = 0.1
        */
        
        d_theta = matrix_multiply_constant(d_theta, alpha);

        //apply d_theta to the control
        cur_joint = endeffector_joint;
        var ind = 0;
        while ( cur_joint != "base" )
        {
            robot.joints[cur_joint].control = d_theta[ind][0];
            ind += 1;
            var plink = robot.joints[cur_joint].parent;
            if ( plink == "base")
                    cur_joint = "base";
            else cur_joint = robot.links[plink].parent;
        }

    //}

}