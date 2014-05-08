//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

matrix_multiply
matrix_transpose
vector_normalize
vector_cross
generate_identity
generate_translation_matrix
generate_rotation_matrix_X
generate_rotation_matrix_Y
generate_rotation_matrix_Z
*/

function vectortomatrix(vec)
{
	mt = [
		 [1, 0, 0, vec[0][0]],
		 [0, 1, 0, vec[1][0]],
		 [0, 0, 1, vec[2][0]],
		 [0, 0, 0, 1]
	];
	return mt;
}


function postomatrix(pos)
{
	mt = [
	 	 [1, 0, 0, pos[0]],
		 [0, 1, 0, pos[1]],
		 [0, 0, 1, pos[2]],
		 [0, 0, 0, 1]
	];

	return mt;
}

function matrixtovector(mt)
{
	vec = [mt[0][0], mt[1][0], mt[2][0]];

	return vec;
}

function matrixtovector1(mt)
{
	vec = [mt[0][3], mt[1][3], mt[2][3]];

	return vec;
}

function vector_substraction(vec1, vec2)
{
	vec = [vec1[0]-vec2[0], vec1[1]-vec2[1], vec1[2]-vec2[2]];
	return vec;
}

function matrix_multiply_constant(mt, c)
{
	for (var i = 0; i < mt.length; i++)
	{
		for (var j = 0; j < mt[0].length; j++ )
			mt[i][j] = mt[i][j] * c;
	}
	return mt;
}


function matrix_multiply(mt1, mt2)
{
	var flag = false;
	//if ( mt2.length == 4 && mt2[0].length == 6) flag = true;
	mt = [];
	for (var i = 0; i < mt1.length; i++)
	{
		tmp = [];
		//if (flag) console.log(i);
		for (var j = 0; j < mt2[0].length; j++)
		{
			sum = 0;
			for (var k = 0; k < mt2.length; k++) 
				sum = sum + mt1[i][k] * mt2[k][j];
			tmp[j] = sum;
		}
		mt.push(tmp);
	}
	
	return mt
}

function matrix_transpose(mt)
{
	n = mt.length;
	m = mt[0].length;
	mtt = [];
	
	//console.log(mt);
	for (var i = 0 ; i < m; i++)
	{
		mtt[i] = [];
		for (var j = 0 ; j < n; j++)
		{
			mtt[i][j] = mt[j][i];
		}
	}



	return mtt;
}

function vector_normalize(vec)
{
	var sum = 0;
	for(var i = 0 ; i < vec.length ; i++)
		sum += vec[i];
	for(var i = 0 ; i < vec.length ; i++)
		vec[i] = vec[i]/sum;
	
	return vec;
}

function vector_length(vec)
{
	var sum = 0;

	for(var i = 0 ; i < vec.length ; i++)
		sum += vec[i]*vec[i];

	return Math.sqrt(sum);
}

function vector_cross(vec1, vec2)
{
	var vec = [];
	vec[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1];
	vec[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2];
	vec[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];

	return vec;
}

function generate_identity(n)
{
	mt = [];
	for (var i = 0 ; i < n ; i++)
	{
		mt[i] = []
		for (var j = 0 ; j < n ; j++)
		{
			if ( i == j ) mt[i][j] = 1;
			else mt[i][j] = 0;
		}
	}
	return mt;
}

function generate_translation_matrix(x, y, z)
{
	mt = [
		 [1, 0, 0, x],
		 [0, 1, 0, y],
		 [0, 0, 1, z],
		 [0, 0, 0, 1]
	];
	
	return mt;
}

function clean_rotation_matrix(mt)
{
	mt = [
		 [mt[0][0], mt[0][1], mt[0][2], 0],
		 [mt[1][0], mt[1][1], mt[1][2], 0],
		 [mt[2][0], mt[2][1], mt[2][2], 0],
		 [0, 0, 0, 1]
	];

	return mt;
}

function generate_rotation_matrix(theta_x, theta_y, theta_z)
{
	rx = generate_rotation_matrix_X(theta_x);
	ry = generate_rotation_matrix_Y(theta_y);
	rz = generate_rotation_matrix_Z(theta_z);
	
	return matrix_multiply(matrix_multiply(rx, ry), rz);
}

function generate_rotation_matrix_X(theta_x)
{
	mtx =	[
			[1, 0, 0, 0], 
			[0, Math.cos(theta_x), -Math.sin(theta_x), 0],
			[0, Math.sin(theta_x), Math.cos(theta_x), 0],
			[0, 0, 0, 1]
		];
			
	return mtx;
}

function generate_rotation_matrix_Y(theta_y)
{
	mty = [
			[Math.cos(theta_y), 0, Math.sin(theta_y), 0], 
			[0, 1, 0, 0],
			[-Math.sin(theta_y), 0, Math.cos(theta_y), 0],
			[0, 0, 0, 1]
		];
	return mty;
}

function generate_rotation_matrix_Z(theta_z)
{
	mtz =[
			[Math.cos(theta_z), -Math.sin(theta_z), 0, 0], 
			[Math.sin(theta_z), Math.cos(theta_z), 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1]
		];
	return mtz;
}