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


function matrix_multiply(mt1, mt2)
{
	mt = [];
	for (var i = 0; i < mt1.length; i++)
	{
		tmp = [];
		for (var j = 0; j < mt2[i].length; j++)
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

function generate_translation_matrix(t1, t2)
{
	return matrix_multiply(t1, t2);
}