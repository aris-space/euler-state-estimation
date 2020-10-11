#include "../../Inc/Util/math_utils.h"


void eye(int dim, float A[dim][dim]) {
	for(int i = 0; i < dim; i++){
        for(int j = 0; j < dim; j++){
            if (i == j){
				A[i][j] = 1;
			} else {
				A[i][j] = 0;
			}
        }
    }
}

void transpose(int m, int n, float A[m][n], float A_T[n][m]) {
	/* Get Transpose */
	for(int i = 0; i < m; i++){
		for(int j = 0; j < n; j++){
			A_T[j][i] = A[i][j];
		}
	}
}

void vecadd(int n, float a[n], float b[n], float c[n]) {
	for(int i = 0; i < n; i++){
		c[i] = a[i] + b[i];
	}
}

void vecsub(int n, float a[n], float b[n], float c[n]) {
	for(int i = 0; i < n; i++){
		c[i] = a[i] - b[i];
	}
}

void scalarvecprod(int n, float scalar, float a[n], float b[n]){
    for(int i = 0; i < n; i++){
        b[i] = scalar * a[i];
    }
}

void matadd(int m, int n, float A[m][n], float B[m][n], float C[m][n]) {
	for(int i = 0; i < m; i++){
        for(int j = 0; j < n; j++){
            C[i][j] =  A[i][j] + B[i][j];
        }
    }
}

void matsub(int m, int n, float A[m][n], float B[m][n], float C[m][n]) {
	for(int i = 0; i < m; i++){
        for(int j = 0; j < n; j++){
            C[i][j] =  A[i][j] - B[i][j];
        }
    }
}

void matmul(int m, int n, int o, float A[m][n], float B[n][o], float C[m][o], bool reset) {
	if (reset) {
		memset(C, 0, m * o * sizeof(C[0][0]));
	}
	for(int i = 0; i < m; i++){
        for(int j = 0; j < o; j++){
            for(int k = 0; k < n; k++){
                C[i][j] +=  A[i][k] * B[k][j];
            }
        }
    }
}

void matvecprod(int m, int n, float A[m][n], float b[n], float c[m], bool reset) {
	if (reset) {
		memset(c, 0, m * sizeof(c[0]));
	}
	for(int i = 0; i < m; i++){
        for(int j = 0; j < n; j++){
            c[i] += A[i][j] * b[j];
        }
    }
}

void scalarmatprod(int m, int n, float scalar, float A[m][n], float B[m][n]) {
	for(int i = 0; i < m; i++){
        for(int j = 0; j < n; j++){
            B[i][j] = scalar * A[i][j];
        }
    }
}

float vecsum(int n, float a[n]) {
	float sum = 0;
	for(int i = 0; i < n; i++){
        sum += a[i];
    }
	return sum;
}

/* Function to get cofactor of A[p][q] in temp[][]. n is current dimension of A[][] */
/* https://www.geeksforgeeks.org/adjoint-inverse-matrix/ */
void cofactor(int dim, float A[dim][dim], float temp[dim][dim], int p, int q, int n) 
{ 
    int i = 0, j = 0; 
  
    // Looping for each element of the matrix 
    for (int row = 0; row < n; row++) 
    { 
        for (int col = 0; col < n; col++) 
        { 
            //  Copying into temporary matrix only those element 
            //  which are not in given row and column 
            if (row != p && col != q) 
            { 
                temp[i][j++] = A[row][col]; 
  
                // Row is filled, so increase row index and 
                // reset col index 
                if (j == n - 1) 
                { 
                    j = 0; 
                    i++; 
                } 
            } 
        } 
    } 
} 
  
/* Recursive function for finding determinant of matrix. n is current dimension of A[][]. */
/* https://www.geeksforgeeks.org/adjoint-inverse-matrix/ */
float determinant(int dim, float A[dim][dim], int n) 
{ 
    float D = 0; // Initialize result 
  
    //  Base case : if matrix contains single element 
    if (n == 1) 
        return A[0][0]; 
  
    float temp[dim][dim]; // To store cofactors 
  
    int sign = 1;  // To store sign multiplier 
  
     // Iterate for each element of first row 
    for (int f = 0; f < n; f++) 
    { 
        // Getting Cofactor of A[0][f] 
        cofactor(dim, A, temp, 0, f, n); 
        D += sign * A[0][f] * determinant(dim, temp, n - 1); 
  
        // terms are to be added with alternate sign 
        sign = -sign; 
    } 
  
    return D; 
} 
  
/* Function to get adjoint of A[dim][dim] in adj[dim][dim]. */
/* https://www.geeksforgeeks.org/adjoint-inverse-matrix/ */
void adjoint(int dim, float A[dim][dim], float adj[dim][dim]) 
{ 
    if (dim == 1) 
    { 
        adj[0][0] = 1; 
        return; 
    } 
  
    // temp is used to store cofactors of A[][] 
    int sign = 1;
	float temp[dim][dim]; 
  
    for (int i=0; i<dim; i++) 
    { 
        for (int j=0; j<dim; j++) 
        { 
            // Get cofactor of A[i][j] 
            cofactor(dim, A, temp, i, j, dim); 
  
            // sign of adj[j][i] positive if sum of row 
            // and column indexes is even. 
            sign = ((i+j)%2==0)? 1: -1; 
  
            // Interchanging rows and columns to get the 
            // transpose of the cofactor matrix 
            adj[j][i] = (sign)*(determinant(dim, temp, dim-1)); 
        } 
    } 
} 
  
/* Function to calculate and store inverse, returns false if matrix is singular */
/* https://www.geeksforgeeks.org/adjoint-inverse-matrix/ */
bool inverse(int dim, float A[dim][dim], float A_inv[dim][dim], float lambda) 
{ 
	/* add damping factor to avoid singularities. */
	/* if no damping is required set lambda to 0.0 */
	float A_dash[dim][dim];
    memcpy(A_dash, A, dim * dim * sizeof(A[0][0]));
    for (int i=0; i<dim; i++) {
        A_dash[i][i] = A_dash[i][i] + lambda * lambda; 
	}

    // Find determinant of A[][] 
    float det = determinant(dim, A_dash, dim);

    if (det == 0) 
    { 
        printf("Singular matrix, can't find its inverse\n"); 
        return false; 
    } 
  
    // Find adjoint 
    float adj[dim][dim]; 
    adjoint(dim, A_dash, adj); 
  
    // Find Inverse using formula "inverse(A) = adj(A)/det(A)" 
    for (int i=0; i<dim; i++) {
        for (int j=0; j<dim; j++) {
            A_inv[i][j] = adj[i][j] / ((float) det); 
		}
	}
  
    return true; 
} 

/* Damped Moore-Penrose pseudo-inverse - ETHZ Robot Dynamics Lecture notes */
bool pseudo_inverse(int m, int n, float A[m][n], float A_inv[n][m], float lambda) { 
	float A_T[n][m];
	transpose(m, n, A, A_T);

	bool inversible;
    if (m >= n) {
		/* we need to calculate left pseudo-inverse */
		float A_int[n][n];
		matmul(n, m, n, A_T, A, A_int, true);
		
		float A_int_inv[n][n];
		inversible = inverse(n, A_int, A_int_inv, lambda);
		if (inversible == true){
			matmul(n, n, m, A_int_inv, A_T, A_inv, true);
			return true;
		}
	} else {
		/* we need to calculate right pseudo-inverse */
		float A_int[m][m];
		matmul(m, n, m, A, A_T, A_int, true);
		
		float A_int_inv[m][m];
		inversible = inverse(m, A_int, A_int_inv, lambda);
		if (inversible == true){
			matmul(n, m, m, A_T, A_int_inv, A_inv, true);
			return true;
		}
	}
	return false;
}

/* the inverse of a matrix which has only diagonal elements */
void diag_inverse(int n, float A[n][n], float inverse[n][n], float lambda) {
    for(int i = 0; i < n; i++){
		inverse[i][i] = 1 / (A[i][i] + lambda);
	}
}

/* computes in-place the inverse of the lower triangular matrix L */
/* http://www.mymathlib.com/matrices/linearsystems/triangular.html */
int lower_triangular_inverse(int n, float *L) {
   int i, j, k;
   float *p_i, *p_j, *p_k;
   float sum;

    /* Invert the diagonal elements of the lower triangular matrix L. */
    for (k = 0, p_k = L; k < n; p_k += (n + 1), k++) {
        if (*p_k == 0.0) return -1;
        else *p_k = 1.0 / *p_k;
    }
    
    /* Invert the remaining lower triangular matrix L row by row. */
    for (i = 1, p_i = L + n; i < n; i++, p_i += n) {
        for (j = 0, p_j = L; j < i; p_j += n, j++) {
            sum = 0.0;
            for (k = j, p_k = p_j; k < i; k++, p_k += n)
                sum += *(p_i + k) * *(p_k + j);
            *(p_i + j) = - *(p_i + i) * sum;
        }
    }
    
    return 0;
}

/* computes the cholesky decomposition */
/* https://rosettacode.org/wiki/Cholesky_decomposition#C */
void cholesky(int n, float A[n][n], float L[n][n]) {
    memset(L, 0, n * n * sizeof(L[0][0]));
    for (int i = 0; i < n; i++)
        for (int j = 0; j < (i+1); j++) {
            float s = 0;
            for (int k = 0; k < j; k++)
                s += L[i][k] * L[j][k];
            L[i][j] = (i == j) ? sqrtf(A[i][i] - s) : (1.0 / L[j][j] * (A[i][j] - s));
        }
}

/* computes the inverse of a Hermitian, positive-definite matrix of dimension n x n using cholesky decomposition*/
/* Krishnamoorthy, Aravindh, and Deepak Menon. "Matrix inversion using Cholesky decomposition." */
/* 2013 signal processing: Algorithms, architectures, arrangements, and applications (SPA). IEEE, 2013. */
/* the inverse has a big O complexity of n^3 */
void cholesky_inverse(int n, float A[n][n], float inverse[n][n], float lambda) {
    /* add damping factor to avoid singularities. */
	/* if no damping is required set lambda to 0.0 */
	float A_dash[n][n];
    memcpy(A_dash, A, n * n * sizeof(A[0][0]));
    for (int i = 0; i < n; i++) {
        A_dash[i][i] = A_dash[i][i] + lambda * lambda; 
	}

    /* call cholesky decomposition to get lower triangular matrix L */
    float L[n][n];
    cholesky(n, A_dash, L);

    /* compute lower triangular inverse in-place */
    lower_triangular_inverse(n, &L[0][0]);

    /* compute matrix inverse A_inv = L_T^(-1) * L^(-1) */
    memset(inverse, 0, n * n * sizeof(inverse[0][0]));
    for(int i = 0; i < n; i++){
        for(int j = 0; j < n; j++){
            for(int k = max(i, j); k < n; k++){
                inverse[i][j] +=  L[k][i] * L[k][j];
            }
        }
    }
}

void interpolate(float y[2], float x[2], float xp, float *yp){
    *yp = y[0] + ((y[1]-y[0])/(x[1]-x[0])) * (xp - x[0]);
    if (xp < x[0]){
        *yp = y[0];
    }
    if (xp > x[1]){
        *yp = y[1];
    }
}

//----------------------------------------------------
// SOURCE: https://github.com/natedomin/polyfit/blob/master/polyfit.c
// 
// METHOD:  polyfit
//
// INPUTS:  dependentValues[0..(countOfElements-1)]
//          independentValues[0...(countOfElements-1)]
//          countOfElements
//          order - Order of the polynomial fitting
//
// OUTPUTS: coefficients[0..order] - indexed by term
//               (the (coef*x^3) is coefficients[3])
//
//----------------------------------------------------
int polyfit(const float* const dependentValues,
            const float* const independentValues,
            unsigned int countOfElements,
            unsigned int order,
            double* coefficients)
{
    // Declarations...
    // ----------------------------------
    enum {maxOrder = 5};
    
    double B[maxOrder+1] = {0.0f};
    double P[((maxOrder+1) * 2)+1] = {0.0f};
    double A[(maxOrder + 1)*2*(maxOrder + 1)] = {0.0f};

    double x, y, powx;

    unsigned int ii, jj, kk;

    // Verify initial conditions....
    // ----------------------------------

    // This method requires that the countOfElements > 
    // (order+1) 
    if (countOfElements <= order)
        return -1;

    // This method has imposed an arbitrary bound of
    // order <= maxOrder.  Increase maxOrder if necessary.
    if (order > maxOrder)
        return -1;

    // Begin Code...
    // ----------------------------------

    // Identify the column vector
    for (ii = 0; ii < countOfElements; ii++)
    {
        x    = (double) dependentValues[ii];
        y    = (double) independentValues[ii];
        powx = 1;

        for (jj = 0; jj < (order + 1); jj++)
        {
            B[jj] = B[jj] + (y * powx);
            powx  = powx * x;
        }
    }

    // Initialize the PowX array
    P[0] = countOfElements;

    // Compute the sum of the Powers of X
    for (ii = 0; ii < countOfElements; ii++)
    {
        x    = (double) dependentValues[ii];
        powx = (double) dependentValues[ii];

        for (jj = 1; jj < ((2 * (order + 1)) + 1); jj++)
        {
            P[jj] = P[jj] + powx;
            powx  = powx * x;
        }
    }

    // Initialize the reduction matrix
    //
    for (ii = 0; ii < (order + 1); ii++)
    {
        for (jj = 0; jj < (order + 1); jj++)
        {
            A[(ii * (2 * (order + 1))) + jj] = P[ii+jj];
        }

        A[(ii*(2 * (order + 1))) + (ii + (order + 1))] = 1;
    }

    // Move the Identity matrix portion of the redux matrix
    // to the left side (find the inverse of the left side
    // of the redux matrix
    for (ii = 0; ii < (order + 1); ii++)
    {
        x = A[(ii * (2 * (order + 1))) + ii];
        if (x != 0)
        {
            for (kk = 0; kk < (2 * (order + 1)); kk++)
            {
                A[(ii * (2 * (order + 1))) + kk] = 
                    A[(ii * (2 * (order + 1))) + kk] / x;
            }

            for (jj = 0; jj < (order + 1); jj++)
            {
                if ((jj - ii) != 0)
                {
                    y = A[(jj * (2 * (order + 1))) + ii];
                    for (kk = 0; kk < (2 * (order + 1)); kk++)
                    {
                        A[(jj * (2 * (order + 1))) + kk] = 
                            A[(jj * (2 * (order + 1))) + kk] -
                            y * A[(ii * (2 * (order + 1))) + kk];
                    }
                }
            }
        }
        else
        {
            // Cannot work with singular matrices
            return -1;
        }
    }

    // Calculate and Identify the coefficients
    for (ii = 0; ii < (order + 1); ii++)
    {
        for (jj = 0; jj < (order + 1); jj++)
        {
            x = 0;
            for (kk = 0; kk < (order + 1); kk++)
            {
                x = x + (A[(ii * (2 * (order + 1))) + (kk + (order + 1))] *
                    B[kk]);
            }
            coefficients[ii] = x;
        }
    }

    return 0;
}

void discretize(float frequency, int n, int m, float A[n][n], float B[n][m], float Ad[n][n], float Bd[n][m]) {
    // Tustin transform might not work for our application
    /*
    float lambda = 0.00001f; // todo: this value still needs to be hand-tuned
    float A_inv[2][2] = {0};
    float I[2][2] = {0};
    float half_A_T[2][2] = {0};
    float I_plus_half_A_T[2][2] = {0};
    float I_minus_half_A_T[2][2] = {0};
    float I_minus_half_A_T_inv[2][2] = {0};
    float e_A_T_minus_I[2][2] = {0};
    float A_inv_e_A_T_minus_I[2][2] = {0};
    const bool check1 = inverse(2, A, A_inv, lambda); // todo: should we implement something, when one of the checks gets false
    eye(2, I);
    scalarmatprod(2, 2, 0.5f*1.0f/CONTROLLER_SAMPLING_FREQ, A, half_A_T);
    matadd(2, 2, I , half_A_T, I_plus_half_A_T);
    matsub(2, 2, I, half_A_T, I_minus_half_A_T);
    const bool check2 = inverse(2, I_minus_half_A_T, I_minus_half_A_T_inv, lambda);
    matmul(2, 2, 2, I_plus_half_A_T, I_minus_half_A_T_inv, Ad, true);
    matsub(2, 2, Ad, I, e_A_T_minus_I);
    matmul(2, 2, 2, A_inv, e_A_T_minus_I, A_inv_e_A_T_minus_I, true);
    matvecprod(2, 2, A_inv_e_A_T_minus_I, B, Bd, true);
    */

   /* forward Euler method */

    /* Computation of Ad */
    float eye_matrix[n][n];
    memset(&eye_matrix, 0, sizeof(eye_matrix));
    eye(n, eye_matrix);

    float AT[n][n];
    memset(&AT, 0, sizeof(AT));
    
    scalarmatprod(n, n, 1.0f / frequency, A, AT);
    matadd(n, n, eye_matrix, AT, Ad);

    /* Computation of Bd */
    scalarmatprod(n, m, 1.0f / frequency, B, Bd);
}

void body_to_world_rotation(float roll_angle, float pitch_angle, float yaw_angle, float vector[3], float rotated_vector[3]) {
    /* inputs: roll angle phi, pitch angle theta, yaw angle psi in radians, vector in body coordinate system */
    /* we are using the aerospace convention of roll-pitch-yaw angles of Tait-Bryan angles (z-y'-x'') */
    /* DIN 9300 https://de.wikipedia.org/wiki/Eulersche_Winkel#Roll-,_Nick-_und_Gierwinkel:_z-y′-x″-Konvention */
    float phi = roll_angle;
    float theta = pitch_angle;
    float psi = yaw_angle;

    float R[3][3] = {{cosf(theta)*cosf(psi), sinf(phi)*sinf(theta)*cosf(psi)-cosf(phi)*sinf(psi), cosf(phi)*sinf(theta)*cosf(psi)+sinf(phi)*sinf(psi)},
                     {cosf(theta)*sinf(psi), sinf(phi)*sinf(theta)*sinf(psi)+cosf(phi)*cosf(psi), cosf(phi)*sinf(theta)*sinf(psi)-sinf(phi)*cosf(psi)},
                     {-sinf(theta), sinf(phi)*cosf(theta), cosf(phi)*cosf(theta)}};

    matvecprod(3, 3, R, vector, rotated_vector, true);
}

void world_to_body_rotation(float roll_angle, float pitch_angle, float yaw_angle, float vector[3], float rotated_vector[3]) {
    /* inputs: roll angle phi, pitch angle theta, yaw angle psi in radians, vector in body coordinate system */
    /* we are using the aerospace convention of roll-pitch-yaw angles of Tait-Bryan angles (z-y'-x'') */
    /* DIN 9300 https://de.wikipedia.org/wiki/Eulersche_Winkel#Roll-,_Nick-_und_Gierwinkel:_z-y′-x″-Konvention */
    float phi = roll_angle;
    float theta = pitch_angle;
    float psi = yaw_angle;

    float R[3][3] = {{cosf(theta)*cosf(psi), cosf(theta)*sinf(psi), -sinf(theta)},
                     {sinf(phi)*sinf(theta)*cosf(psi)-cosf(phi)*sinf(psi), sinf(phi)*sinf(theta)*sinf(psi)+cosf(phi)*cosf(psi), sinf(phi)*cosf(theta)},
                     {cosf(phi)*sinf(theta)*cosf(psi)+sinf(phi)*sinf(psi), cosf(phi)*sinf(theta)*sinf(psi)-sinf(phi)*cosf(psi), cosf(phi)*cosf(theta)}};

    matvecprod(3, 3, R, vector, rotated_vector, true);
}