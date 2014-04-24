//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

quaternion_from_axisangle
quaternion_normalize
quaternion_multiply
quaternion_to_rotation_matrix
*/


function quaternion_from_axisangle(axis, angle)
{
	q = [];
	q[0] = Math.cos(angle/2);
	q[1] = axis[0] * Math.sin(angle/2);
	q[2] = axis[1] * Math.sin(angle/2);
	q[3] = axis[2] * Math.sin(angle/2);

	return q;
}

function quaternion_normalize(q)
{
	denominator = Math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q[0] = q[0] / denominator;
	q[1] = q[1] / denominator;
	q[2] = q[2] / denominator;
	q[3] = q[3] / denominator;

	return q;
}

function quaternion_multiply(q1, q2)
{
	q = [];
	q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]
	q[1] = q1[0] * q2[1] + q1[1] * q2[0] - q1[2] * q2[3] + q1[3] * q2[2]
	q[2] = q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0] - q1[3] * q2[1]
	q[3] = q1[0] * q2[3] - q1[1] * q2[2] + q1[2] * q2[1] + q1[3] * q2[0]

	return q
}

function quaternion_to_rotation_matrix(q)
{
	q = quaternion_normalize(q);

	mt = [
		 [1 - 2 * (q[2]*q[2] + q[3]*q[3]), 2 * (q[1]*q[2] - q[0]*q[3]), 2*(q[0]*q[2] + q[1]*q[3]), 0],
		 [2*(q[1]*q[2] + q[0]*q[3]), 1 - 2*(q[1]*q[1] + q[3]*q[3]), 2*(q[2]*q[3] - q[0]*q[1]), 0],
		 [2*(q[1]*q[3] - q[0]*q[2]), 2 * (q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]), 0],
		 [0, 0, 0, 1]
	];

	return mt;
}