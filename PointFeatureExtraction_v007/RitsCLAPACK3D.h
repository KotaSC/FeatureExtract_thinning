///////////////////////////
///// RitsCLAPACK3D.h /////
///////////////////////////

#if !defined  RITS_CLAPACK_3D_HH
#define       RITS_CLAPACK_3D_HH

#include <iostream> // cout
#include <Accelerate/Accelerate.h> //CLAPACK
#include <cstdio>
#include <cmath>

#define DIM 3

//#define DEBUG_1

//--------------------//
class RitsCLAPACK3D  {
//--------------------//
 public:
  RitsCLAPACK3D (void) {}
  static void EigenValues (double M[][DIM], double lambda[DIM], double u_vec3[DIM] );

 private:
  
};


//-----
inline void   
RitsCLAPACK3D::EigenValues (double M[][DIM], double lambda[DIM], double u_vec3[DIM] )
{
  const double EPSILON = 1.0e-6;

  int N = DIM;

  // local variables
  double A[N*N]; //NxN matrix
  double wr[N]; // wr[i] is real part of i-th eiven value
  double wi[N]; // wi[i] is imaginary part of i-th eiven value
  double work[4*N]; // working area:
                    //  Its size should be larger than 4*N
  double vl[N*N]; // unused array
  double vr[N*N]; // eigen vector
                // vr[0] = x1
                // vr[1] = y1
                // vr[2] = x2
                // vr[3] = y2
  int n=N;    // number of rows of matrix A
  int lda=N;  // number of colums of matrix A
  int ldvl=N; // dim of vl[]
  int ldvr=N; // dim of vr[]
  int lwork = 4*N; // working area size:
                   //  This size should be consistent with
                   //  the size of array work[] above.
  int info; // status (0: success)
  char jobvl = 'N'; // Left eigen value is not calculated.
  char jobvr = 'V'; // Right eigen value is calculated.

  // Matrix: A[0] = A11, A[1]=A21, ...(raw major)
  //   A[0]=M[0][0]=1, A[3]=M[0][1]=0, A[6]=M[0][2]=-1;
  //   A[1]=M[1][0]=1, A[4]=M[1][1]=2, A[7]=M[1][2]= 1;
  //   A[2]=M[2][0]=2, A[5]=M[2][1]=2, A[8]=M[2][2]= 3;
  for( int i=0; i<n; i++ ) {
    for( int j=0; j<n; j++ ) {
      A[i + j*n] = M[i][j];
    }//j 
  }//i

  // Calc eigen values and eigen vectors
  dgeev_( &jobvl, &jobvr, &n, A, &lda, wr, wi, vl, &ldvl, vr, &ldvr, work, &lwork, &info);

  // Backup the result 
  double lambda_tmp[DIM];
  for(int i=0;i<n;i++){
    lambda_tmp[i] = wr[i];
  }//for(i)

  // Sort the obtained eigen values
  int index_final = 2;
  if        ( lambda_tmp [0] >= lambda_tmp[1] && lambda_tmp[1] >= lambda_tmp[2] ) {
    lambda[0] = lambda_tmp[0];
    lambda[1] = lambda_tmp[1];
    lambda[2] = lambda_tmp[2];

    index_final = 2;

  } else if ( lambda_tmp [0] >= lambda_tmp[2] && lambda_tmp[2] >= lambda_tmp[1] ) {
    lambda[0] = lambda_tmp[0];
    lambda[1] = lambda_tmp[2];
    lambda[2] = lambda_tmp[1];

    index_final = 1;

  } else if ( lambda_tmp [1] >= lambda_tmp[0] && lambda_tmp[0] >= lambda_tmp[2] ) {
    lambda[0] = lambda_tmp[1];
    lambda[1] = lambda_tmp[0];
    lambda[2] = lambda_tmp[2];

    index_final = 2;

  } else if ( lambda_tmp [1] >= lambda_tmp[2] && lambda_tmp[2] >= lambda_tmp[0] ) {
    lambda[0] = lambda_tmp[1];
    lambda[1] = lambda_tmp[2];
    lambda[2] = lambda_tmp[0];

    index_final = 0;

  } else if ( lambda_tmp [2] >= lambda_tmp[0] && lambda_tmp[0] >= lambda_tmp[1] ) {
    lambda[0] = lambda_tmp[2];
    lambda[1] = lambda_tmp[0];
    lambda[2] = lambda_tmp[1];

    index_final = 1;

  } else if ( lambda_tmp [2] >= lambda_tmp[1] && lambda_tmp[1] >= lambda_tmp[0] ) {
    lambda[0] = lambda_tmp[2];
    lambda[1] = lambda_tmp[1];
    lambda[2] = lambda_tmp[0];

    index_final = 0;

  }

  // Eigen function belonging to lambda[2]
  double length2 = 0.0;
  for(int a=0;a<n;a++){
    u_vec3[ a ] = vr[ n*index_final + a ] ;
    length2 += u_vec3[ a ] * u_vec3[ a ];
  }
  double length = std::sqrt(length2) ;

  if ( length > EPSILON ) {
    for( int a=0;a<n;a++ ){
      u_vec3[ a ] /= length;
    }
  }

#if defined DEBUG_1

  // Display status
  std::cout << "info =" << info << std::endl;

  // Display result
  for(int i=0;i<n;i++){
    std::cout << "eigen_value " << (i+1) << ": " ;
    std::cout << wr[i] << "+" << wi[i]   << "i" << "\t";
    std::cout << "eigen_vector " << (i+1) << ": " ;
    for(int a=0;a<n;a++){
      std::cout  << vr[n*i + a] << "  ";
    }//for(a)
    std::cout << std::endl;
  }//for(i)

#endif

}// RitsCLAPACK3D::EigenValue ()

/* 実行結果
  double M[][N] = {
    {1, 0, -1},
    {1, 2,  1},
    {2, 2,  3},
  };

info=0
eigen_value 1: 2+0i eigen_vector 1: -0.666667 0.333333 0.666667 
eigen_value 2: 3+0i eigen_vector 2: -0.408248 0.408248 0.816497 
eigen_value 3: 1+2i eigen_vector 3: 0.707107 -0.707107 1.40433e-16 
*/

#endif
// end of RitsCLAPACK.h
