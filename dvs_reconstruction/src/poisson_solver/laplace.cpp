/*
 * ---------------------------------------------------------------------
 * Copyright (C) 2012, 2014 Tino Kluge (ttk448 at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <boost/multi_array.hpp>
#include <fftw3.h>
#include "dvs_reconstruction/poisson_solver/laplace.h"

#ifdef TIME_REPORT
#include <time.h>
#include <sys/time.h>
namespace pde
{

double stoptime(void)
{
   struct timeval t;
   gettimeofday(&t,NULL);
   return (double) t.tv_sec + t.tv_usec/1000000.0;
}
}
#endif // TIME_REPORT


namespace
{

// convenience function: square
inline double sqr(double x)
{
   return x*x;
}

} // namespace


// 2d-array utility functions
// --------------------------
namespace arr
{

// helper function: [0,1] uniform random generator
double runif()
{
   return (double) rand()/(RAND_MAX+1.0);
}
// fill vector with random values
void runif(std::vector<double>& X)
{
   size_t n=X.size();
   for(size_t i=0; i<n; i++) {
      X[i]=runif();
   }
}
// fill 2d-array with random values
void runif(boost::multi_array<double,2>& X)
{
   size_t n1=X.shape()[0];
   size_t n2=X.shape()[1];
   for(size_t i=0; i<n1; i++) {
      for(size_t j=0; j<n2; j++) {
         X[i][j]=runif();
      }
   }
}

// print the matrix
void print(const boost::multi_array<double,2>& X)
{
   size_t n1=X.shape()[0];
   size_t n2=X.shape()[1];
   for(size_t i=0; i<n1; i++) {
      for(size_t j=0; j<n2; j++) {
         printf("%9.5f ", X[i][j]);
      }
      printf("\n");
   }
}

// L^2 difference between two vectors
double diff(const std::vector<double>& X,
            const std::vector<double>& Y,
            bool allow_shift)
{

   assert(X.size()==Y.size());
   size_t n=X.size();

   double c=0.0;
   if(allow_shift) {
      c=X[1]-Y[1];
   }
   double sum=0.0;
   for(size_t i=0; i<n; i++) {
      sum+=sqr(X[i]-Y[i]-c);
   }
   return sqrt(sum);
}

// L^2 difference between vector and a constant
double diff(const std::vector<double>& X, double c)
{
   size_t n=X.size();
   double sum=0.0;
   for(size_t i=0; i<n; i++) {
      sum+=sqr(X[i]-c);
   }
   return sqrt(sum);
}

// L^2 difference between two 2d-arrays
// allow_shift: removes a constant shift between 2 matrices,
//    i.e.  comparing X[][] and Y[][] + (X[1][1]-Y[1][1])
// ignore_corner: ignores all 4 corners, X[0][0], ...
double diff(const boost::multi_array<double,2>& X,
            const boost::multi_array<double,2>& Y,
            bool allow_shift, bool ignore_corner)
{

   size_t n1=X.shape()[0];
   size_t n2=X.shape()[1];

   assert(Y.shape()[0]==n1 && n1>1);
   assert(Y.shape()[1]==n2 && n2>1);

   double c=0.0;
   if(allow_shift) {
      c=X[1][1]-Y[1][1];
   }
   double sum=0.0;
   for(size_t i=0; i<n1; i++) {
      for(size_t j=0; j<n2; j++) {
         if( !(ignore_corner==true && (i==0 || i==n1-1) && (j==0 || j==n2-1)) )
            sum+=sqr(X[i][j]-Y[i][j]-c);
      }
   }
   return sqrt(sum);
}

} // namespace arr




namespace pde
{


// sets the number of threads for the fftw solver
int fftw_threads(int n)
{
   int ret=fftw_init_threads();
   fftw_plan_with_nthreads(n);
   return ret;
}
// cleans up memory used by fftw for threads
void fftw_clean()
{
   fftw_cleanup();
   fftw_cleanup_threads();
}

// sets boundary vectors to a uniform value
void set_boundary_to_uniform(
   std::vector<double>& bd1a, std::vector<double>& bd1b,
   std::vector<double>& bd2a, std::vector<double>& bd2b,
   double value, size_t n1, size_t n2)
{

   bd1a.resize(n2);
   bd1b.resize(n2);
   bd2a.resize(n1);
   bd2b.resize(n1);
   for(size_t i=0; i<n2; i++) {
      bd1a[i]=value;
      bd1b[i]=value;
   }
   for(size_t i=0; i<n1; i++) {
      bd2a[i]=value;
      bd2b[i]=value;
   }
}

// saves the 4 boundaries/edges of a 2-d array as individual vectors
// note, the 4 corners/vertices are not saved and the resulting
// vectors are of size n-2
void get_boundary(std::vector<double>& bd1a, std::vector<double>& bd1b,
                  std::vector<double>& bd2a, std::vector<double>& bd2b,
                  const boost::multi_array<double,2>& X,
                  double h1, double h2, types::boundary boundary)
{
   size_t n1=X.shape()[0];
   size_t n2=X.shape()[1];
   assert(n1>2 && n2>2);

   // allocating memory for boundary vectors
   bd1a.resize(n2-2);
   bd1b.resize(n2-2);
   bd2a.resize(n1-2);
   bd2b.resize(n1-2);

   // reading boundary values
   if(boundary==types::Dirichlet) {
      for(size_t i=1; i<n2-1; i++) {
         bd1a[i-1] = X[0][i];
         bd1b[i-1] = X[n1-1][i];
      }
      for(size_t i=1; i<n1-1; i++) {
         bd2a[i-1] = X[i][0];
         bd2b[i-1] = X[i][n2-1];
      }
   } else if(boundary==types::Neumann) {
      for(size_t i=1; i<n2-1; i++) {
         bd1a[i-1] = (X[0][i]-X[2][i]) / (2.0*h1);
         bd1b[i-1] = (X[n1-1][i]-X[n1-3][i]) / (2.0*h1);
      }
      for(size_t i=1; i<n1-1; i++) {
         bd2a[i-1] = (X[i][0]-X[i][2]) / (2.0*h2);
         bd2b[i-1] = (X[i][n2-1]-X[i][n2-3]) / (2.0*h2);
      }
   } else {
      assert(false);
   }
}


// set boundary of matrix X according to conditions
// can either add boundaries to the array (resize), or overwrite the
// current boundary values
// boundary vectors shall not contain the 4 corners (vertices), ie need to
// be of size n-2 of the resulting array
void set_boundary(boost::multi_array<double,2>& X, double h1, double h2,
                  const std::vector<double>& bd1a, const std::vector<double>& bd1b,
                  const std::vector<double>& bd2a, const std::vector<double>& bd2b,
                  types::boundary boundary, bool add)
{

   size_t n1=X.shape()[0];
   size_t n2=X.shape()[1];
   assert(n1>0 && n2>0);

   if(add==true) {
      // add boundary --> grow array
      X.resize(boost::extents[n1+2][n2+2]);  // element preserving resize
      n1=X.shape()[0];
      n2=X.shape()[1];
      // shifting numbers inside the array to make space for boundary
      for(int i=n1-3; i>=0; i--) {
         for(int j=n2-3; j>=0; j--) {
            X[i+1][j+1]=X[i][j];
         }
      }
   }

   // check the boundary vectors are of the correct size
   assert( bd1a.size()==bd1b.size() && bd1a.size()==n2-2 );
   assert( bd2a.size()==bd2b.size() && bd2a.size()==n1-2 );

   // setting boundary values
   if(boundary==types::Dirichlet) {
      for(int i=0; i<(int)n2; i++) {
         // include corner cases, which are otherwise undetermined
         int idx = std::min(std::max(i-1,0),(int)n2-3);
         X[0][i]    = bd1a[idx];
         X[n1-1][i] = bd1b[idx];
      }
      for(size_t i=1; i<n1-1; i++) {
         X[i][0]    = bd2a[i-1];
         X[i][n2-1] = bd2b[i-1];
      }
   } else if(boundary==types::Neumann) {
      for(int i=0; i<(int)n2; i++) {
         // include corner cases, which are otherwise undetermined
         int idx = std::min(std::max(i-1,0),(int)n2-3);
         X[0][i]    = X[2][i]    + 2.0*h1*bd1a[idx];
         X[n1-1][i] = X[n1-3][i] + 2.0*h1*bd1b[idx];
      }
      for(size_t i=1; i<n1-1; i++) {
         X[i][0]    = X[i][2]    + 2.0*h2*bd2a[i-1];
         X[i][n2-1] = X[i][n2-3] + 2.0*h2*bd2b[i-1];
      }
   } else {
      assert(false);
   }
}
// specialised version where all boundaries have the same value
void set_boundary(boost::multi_array<double,2>& X, double h1, double h2,
                  double bdvalue, types::boundary bdcond, bool add)
{
   size_t n1=X.shape()[0];
   size_t n2=X.shape()[1];
   // set the correct size of the boundary vectors
   size_t m1,m2;
   if(add==true) {
      m1=n1;
      m2=n2;
   } else {
      assert(n1>=2 && n2>=2);
      m1=n1-2;
      m2=n2-2;
   }
   // fill boundary vectors
   std::vector<double> bd1a,bd1b,bd2a,bd2b;
   set_boundary_to_uniform(bd1a,bd1b,bd2a,bd2b,bdvalue,m1,m2);

   // call the general routine
   set_boundary(X,h1,h2,bd1a,bd1b,bd2a,bd2b,bdcond,add);
}





// PDE functions to solve the discrete Poisson's Equation
// ------------------------------------------------------

// discrete Gradient operator on a uniform grid
// given u(x,y), it returns DX = a_1 du/dx, DY = a_2 du/dy,
// using one-sided finite differences
// U[i][j]  = u(x_i, y_j), i=0...n-1
// DX[i][j] = a1*(U[i+1][j]-U[i][j])/h, i=0...n-2
void grad(boost::multi_array<double,2>& DX, boost::multi_array<double,2>& DY,
          const boost::multi_array<double,2>& U,
          double a1, double a2, double h1, double h2)
{
   assert(U.shape()[0]>1 && U.shape()[1]>1);
   size_t n1=U.shape()[0]-1;
   size_t n2=U.shape()[1]-1;
   DX.resize(boost::extents[n1][n2]);
   DY.resize(boost::extents[n1][n2]);
   for(size_t i=0; i<n1; i++) {
      for(size_t j=0; j<n2; j++) {
         DX[i][j] = (a1/h1) * (U[i+1][j]-U[i][j]);
         DY[i][j] = (a2/h2) * (U[i][j+1]-U[i][j]);
      }
   }
}
// discrete Gradient operator on a uniform grid
// same as above, but the input U is assumed to contain only inner grid points
// and the boundary grid points are inferred from the boundary condition
// U[i][j] = u(x_i, y_j), i=0...n-1
// DX[i][j] = a1*(U[i][j]-U[i-1][j])/h, i=0...n+1
void grad(boost::multi_array<double,2>& DX, boost::multi_array<double,2>& DY,
          const boost::multi_array<double,2>& U,
          double a1, double a2, double h1, double h2,
          const std::vector<double>& bd1a, const std::vector<double>& bd1b,
          const std::vector<double>& bd2a, const std::vector<double>& bd2b,
          types::boundary boundary)
{
   // this is lazy and very memory inefficient
   // TODO: replace with a direct calculation
   boost::multi_array<double,2> V=U;
   pde::set_boundary(V,h1,h2,bd1a,bd1b,bd2a,bd2b,boundary,true);
   grad(DX,DY,V,a1,a2,h1,h2);
}
// simplified version where only one boundary value can be specified
void grad(boost::multi_array<double,2>& DX, boost::multi_array<double,2>& DY,
          const boost::multi_array<double,2>& U,
          double a1, double a2, double h1, double h2,
          double bdvalue, types::boundary bdcond)
{
   size_t n1=U.shape()[0];
   size_t n2=U.shape()[1];

   // fill boundary vectors
   std::vector<double> bd1a,bd1b,bd2a,bd2b;
   set_boundary_to_uniform(bd1a,bd1b,bd2a,bd2b,bdvalue,n1,n2);

   // call the general routine
   grad(DX,DY,U,a1,a2,h1,h2,bd1a,bd1b,bd2a,bd2b,bdcond);
}

// discrete Divergence operator on a uniform grid
// given u(x,y), v(x,y), it returns f = a_1 du/dx + a_2 dv/dx,
// using one-sided finite differences
void div(boost::multi_array<double,2>& F,
         const boost::multi_array<double,2>& U,
         const boost::multi_array<double,2>& V,
         double a1, double a2, double h1, double h2)
{
   assert(U.shape()[0]==V.shape()[0]);
   assert(U.shape()[1]==V.shape()[1]);
   assert(U.shape()[0]>1 && U.shape()[1]>1);
   size_t n1=U.shape()[0]-1;
   size_t n2=V.shape()[1]-1;
   F.resize(boost::extents[n1][n2]);
   for(size_t i=0; i<n1; i++) {
      for(size_t j=0; j<n2; j++) {
         F[i][j] = (a1/h1) * (U[i+1][j+1]-U[i][j+1])
                   + (a2/h2) * (V[i+1][j+1]-V[i+1][j]);
      }
   }
}

// discrete Laplace operator on a uniform grid
// given u, it returns f = a_1 u_xx + a_2 u_yy
// note, the Laplace operator can only be applied to inner grid points
// and so the result F has two grid points less in each dimension, i.e.
// U[i][j] = u(x_i, y_j)
// F[i][j] = f(x_{i+1}, y_{i+1})
void laplace(boost::multi_array<double,2>& F,
             const boost::multi_array<double,2>& U,
             double a1, double a2, double h1, double h2)
{
   assert(U.shape()[0]>2 && U.shape()[1]>2);
   size_t n1=U.shape()[0]-2;
   size_t n2=U.shape()[1]-2;
   F.resize(boost::extents[n1][n2]);
   for(size_t i=0; i<n1; i++) {
      for(size_t j=0; j<n2; j++) {
         // index [i][j] in F corresponds to [i+1][j+1] in U
         F[i][j] = a1 * (U[i][j+1] - 2.0*U[i+1][j+1] + U[i+2][j+1]) / (h1*h1)
                   + a2 * (U[i+1][j] - 2.0*U[i+1][j+1] + U[i+1][j+2]) / (h2*h2);
      }
   }
}
// discrete Laplace operator on a uniform grid
// the input U is assumed to contain only inner grid points and the
// boundary grid points are inferred from the boundary condition
// U[i][j] = u(x_i, y_j)
// F[i][j] = f(x_i, y_i)
void laplace(boost::multi_array<double,2>& F,
             const boost::multi_array<double,2>& U,
             double a1, double a2, double h1, double h2,
             const std::vector<double>& bd1a, const std::vector<double>& bd1b,
             const std::vector<double>& bd2a, const std::vector<double>& bd2b,
             types::boundary boundary)
{
   size_t n1=U.shape()[0];
   size_t n2=U.shape()[1];
   assert(n1>0 && n2>0);
   assert(bd1a.size()==bd1b.size() && bd1a.size()==n2);
   assert(bd2a.size()==bd2b.size() && bd2a.size()==n1);

   F.resize(boost::extents[n1][n2]);
   for(int i=0; i<(int)n1; i++) {
      for(int j=0; j<(int)n2; j++) {
         double Um1, U0, Up1;       // U[i-1], U[i], U[i+1]

         // first dimension
         Um1 = U[std::max(i-1,0)][j];
         U0  = U[i][j];
         Up1 = U[std::min(i+1,(int)n1-1)][j];
         if(i==0) {
            if(boundary==types::Dirichlet) {
               Um1 = bd1a[j];
            } else if(boundary==types::Neumann) {
               Um1 = U[1][j] + 2.0*h1*bd1a[j];
            }
         } else if(i==(int)n1-1) {
            if(boundary==types::Dirichlet) {
               Up1 = bd1b[j];
            } else if(boundary==types::Neumann) {
               Up1 = U[n1-2][j] + 2.0*h1*bd1b[j];
            }
         }
         F[i][j] = a1 * (Um1 - 2.0*U0 + Up1) / (h1*h1);

         // second dimension
         Um1 = U[i][std::max(j-1,0)];
         Up1 = U[i][std::min(j+1,(int)n2-1)];
         if(j==0) {
            if(boundary==types::Dirichlet) {
               Um1 = bd2a[i];
            } else if(boundary==types::Neumann) {
               Um1 = U[i][1] + 2.0*h2*bd2a[i];
            }
         } else if(j==(int)n2-1) {
            if(boundary==types::Dirichlet) {
               Up1 = bd2b[i];
            } else if(boundary==types::Neumann) {
               Up1 = U[i][n2-2] + 2.0*h2*bd2b[i];
            }
         }
         F[i][j] += a2 * (Um1 - 2.0*U0 + Up1) / (h2*h2);
      }
   }
}
// simplified version where only one boundary value can be specified
void laplace(boost::multi_array<double,2>& F,
             const boost::multi_array<double,2>& U,
             double a1, double a2, double h1, double h2,
             double bdvalue, types::boundary bdcond)
{

   size_t n1=U.shape()[0];
   size_t n2=U.shape()[1];
   assert(n1>0 && n2>0);

   // fill boundary vectors
   std::vector<double> bd1a,bd1b,bd2a,bd2b;
   set_boundary_to_uniform(bd1a,bd1b,bd2a,bd2b,bdvalue,n1,n2);

   // call the general routine
   laplace(F,U,a1,a2,h1,h2,bd1a,bd1b,bd2a,bd2b,bdcond);
}

// Neumann boundary condition with a general right hand side F will not
// have a solution, however by looking at F we can say what the minimum
// L2-norm error will be for 0-Neumann boundary conditions:
// return = min ||Laplace U - F||
// min is taken over all U consistent with 0-Neumann condition
double neumann_error(const boost::multi_array<double,2>& F)
{
   size_t n1=F.shape()[0];
   size_t n2=F.shape()[1];
   assert(n1>1 && n2>1);

   // assuming 0 boundary conditions, otherwise would have to adjust
   // right hand side F as in poisolve()

   // calculate min L2 error ||Laplace U - F|| = |\hat F[0]| * ||EV[0]||
   // where \hat F, are the coordinates of F in EV-space,
   // and can simply be calculated as follows
   double sum=0.0;
   double fac=1.0;
   for(size_t i=0; i<n1; i++) {
      for(size_t j=0; j<n2; j++) {
         fac=1.0;
         if(j==0 || j==n2-1)
            fac*=0.5;
         if(i==0 || i==n1-1)
            fac*=0.5;
         sum+=fac*F[i][j];
      }
   }
   double F00 = sum/((n1-1)*(n2-1));      // \hat F [0][0] (EV space)
   double norm_ev=sqrt((double)(n1*n2));  // EV[0]=(1,...,1) --> norm=sqrt(n)
   double l2_error=F00*norm_ev;
   return l2_error;
}

// given a right hand side F, we can find a constant Neumann-boundary
// value which will have a solution U, so that Laplace U = F,
// returns the boundary value
double neumann_compat(const boost::multi_array<double,2>& F,
                      double a1, double a2, double h1, double h2)
{
   size_t n1=F.shape()[0];
   size_t n2=F.shape()[1];

   double l2_error=neumann_error(F);
   double norm_ev=sqrt((double)(n1*n2));  // EV[0]=(1,...,1) --> norm=sqrt(n)
   double F00 = l2_error/norm_ev;         // \hat F [0][0] (EV space)

   // with non-zero Neumann boundary condition, rhs F is modified,
   // as in poisolve(), so we can calculate the exact boundary
   // value to make the l2_error zero
   double bd = F00 / (2.0*a1/(h1*(n1-1)) + 2.0*a2/(h2*(n2-1)));

   return bd;
}




// solves the 2D Poisson equation: a1 u_xx + a2 u_yy = f
//
// discretised with uniform grid, x_{i+1} = x_i + h1, y_{i+1} = y_i + h2
// input: rhs       F[i][j] = f(x_i, y_j)
// output: solution  U[i][j] = u(x_i, y_j), ie only inner points by default
//
//
//
// boundary condition:
// - lower and upper boundary in dimension 1: bd1a, bd1b
// - lower and upper boundary in dimension 2: bd2a, bd2b
// - e.g. bd1a[i] refers to U[-1][i]
double poisolve( boost::multi_array<double,2>& U,
                 const boost::multi_array<double,2>& F,
                 double a1, double a2, double h1, double h2,
                 const std::vector<double>& bd1a, const std::vector<double>& bd1b,
                 const std::vector<double>& bd2a, const std::vector<double>& bd2b,
                 types::boundary boundary, bool add_boundary_to_solution)
{

#ifdef TIME_REPORT
   double t0=stoptime();
   double t1=t0;
   double t2;
#endif

   size_t n1=F.shape()[0];
   size_t n2=F.shape()[1];
   assert(n1>0 && n2>0);


   // adjust right hand side F with boundary condition (nothing to do for =0)
   boost::multi_array<double,2> rhs = F;
   assert( bd1a.size()==bd1b.size() && bd1a.size()==n2 );
   assert( bd2a.size()==bd2b.size() && bd2a.size()==n1 );
   {
      // factors for boundary adjustment to rhs
      double c1 = 0.0, c2 = 0.0;
      if(boundary==types::Dirichlet) {
         c1=a1/sqr(h1);
         c2=a2/sqr(h2);
      } else if(boundary==types::Neumann) {
         c1=2.0*a1/h1;
         c2=2.0*a2/h2;
      } else {
         assert(false);
      }
      // adjust rhs with boundary conditions
      for(size_t i=0; i<n2; i++) {
         rhs[0][i]    -= c1 * bd1a[i];
         rhs[n1-1][i] -= c1 * bd1b[i];
      }
      for(size_t i=0; i<n1; i++) {
         rhs[i][0]    -= c2 * bd2a[i];
         rhs[i][n2-1] -= c2 * bd2b[i];
      }
   }


#ifdef TIME_REPORT
   t2=stoptime();
   printf("poisolve(): %5.0f ms: rhs boundary conditions\n",(t2-t1)*1000.0);
   t1=t2;
#endif


   // transform rhs into EV space (inverse fft, in-place)
   {
      fftw_plan p;
      double fft_norm=0.0;  // i.e. FFT(FFT(x)) = fft_norm * x

      if(boundary==types::Dirichlet) {
         // DST-I = FFTW_RODFT00 (note, additional factor of 2 in fftw)
         p=fftw_plan_r2r_2d(n1, n2, &(rhs[0][0]), &(rhs[0][0]),
                            FFTW_RODFT00, FFTW_RODFT00, FFTW_ESTIMATE);
         fftw_execute(p);
         fftw_destroy_plan(p);
         fft_norm = 4.0*((n1+1)*(n2+1));

      } else if(boundary==types::Neumann) {
         // EV space is similar to DCT-I but without the factor of 1/2
         // for the 1st and last element

         // DCT-I = REDFT00 (note, additional factor of 2 in fftw)
         // so that DCT-I*DCT-I = 2(n-1) I (and not the usual (n-1)/2)
         p=fftw_plan_r2r_2d(n1, n2, &(rhs[0][0]), &(rhs[0][0]),
                            FFTW_REDFT00, FFTW_REDFT00, FFTW_ESTIMATE);
         fftw_execute(p);
         fftw_destroy_plan(p);

         for(size_t i=0; i<n1; i++) {
            rhs[i][0]    *= 0.5;
            rhs[i][n2-1] *= 0.5;
         }
         for(size_t j=0; j<n2; j++) {
            rhs[0][j]    *= 0.5;
            rhs[n1-1][j] *= 0.5;
         }
         fft_norm = 4.0*((n1-1)*(n2-1));

      } else {
         assert(false);
      }
      // scale so we get the inverse fft
      for(size_t i=0; i<n1; i++) {
         for(size_t j=0; j<n2; j++) {
            rhs[i][j] *= (1.0/fft_norm);      // div is more expensive than mul
         }
      }
   }


#ifdef TIME_REPORT
   t2=stoptime();
   printf("poisolve(): %5.0f ms: rhs to EV space (inv fft)\n",(t2-t1)*1000.0);
   t1=t2;
#endif


   // calculate eigenvalues of the linear operators L
   std::vector<double> lambda1(n1);
   std::vector<double> lambda2(n2);
   if(boundary==types::Dirichlet) {
      for(size_t i=0; i<n1; i++)
         lambda1[i] = -4.0*sqr( sin((M_PI*(i+1))/(2.0*(n1+1))) );
      for(size_t i=0; i<n2; i++)
         lambda2[i] = -4.0*sqr( sin((M_PI*(i+1))/(2.0*(n2+1))) );
   } else if(boundary==types::Neumann) {
      for(size_t i=0; i<n1; i++)
         lambda1[i] = -4.0*sqr( sin((M_PI*i)/(2.0*(n1-1))) );
      for(size_t i=0; i<n2; i++)
         lambda2[i] = -4.0*sqr( sin((M_PI*i)/(2.0*(n2-1))) );
   } else {
      assert(false);
   }


   // solve the equation for U in EV space
   double error=0.0;
   U.resize(boost::extents[n1][n2]);
   for(size_t i=0; i<n1; i++) {
      for(size_t j=0; j<n2; j++) {
         double div = (a1*lambda1[i]/(h1*h1) + a2*lambda2[j]/(h2*h2));
         if(div==0.0) {
            // here we need rhs[i][j] == 0 or else there is no solution
            // calculate the L2-norm of the error, need to know norm of EV
            // we know div==0.0, only for Neumann and EV 0, ||EV[0]||^2=n
            // however, since fftw has an extra factor 2, ||EV[0]||^2=4n
            double norm_ev = 16.0*n1*n2;     // TODO: make this more general
            error+=sqr(rhs[i][j])*norm_ev;
            // U[i][j] is arbitrary here
            U[i][j] = 0.0;
         } else {
            U[i][j] = rhs[i][j] / div;
         }
      }
   }

   // free rhs memory as it's no longer needed
   rhs.resize(boost::extents[0][0]);

#ifdef TIME_REPORT
   t2=stoptime();
   printf("poisolve(): %5.0f ms: solve equation in EV space\n",(t2-t1)*1000.0);
   t1=t2;
#endif


   // transform U from EV space into canonical space (fft, in-place)
   {
      fftw_plan p = nullptr;

      if(boundary==types::Dirichlet) {
         // DST-I = FFTW_RODFT00 (note, additional factor of 2 in fftw)
         // so that DST-I*DST-I = 2(n+1) I (and not the usual (n+1)/2)
         p=fftw_plan_r2r_2d(n1, n2, &(U[0][0]), &(U[0][0]),
                            FFTW_RODFT00, FFTW_RODFT00, FFTW_ESTIMATE);
         fftw_execute(p);

      } else if(boundary==types::Neumann) {
         // EV space is similar to DCT-I but without the factor of 1/2
         // for the 1st and last element

         // apply factor 2
         for(size_t i=0; i<n1; i++) {
            U[i][0]    *= 2.0;
            U[i][n2-1] *= 2.0;
         }
         for(size_t j=0; j<n2; j++) {
            U[0][j]    *= 2.0;
            U[n1-1][j] *= 2.0;
         }
         // DCT-I = REDFT00 (note, additional factor of 2 in fftw)
         p=fftw_plan_r2r_2d(n1, n2, &(U[0][0]), &(U[0][0]),
                            FFTW_REDFT00, FFTW_REDFT00, FFTW_ESTIMATE);
         fftw_execute(p);

      } else {
         assert(false);
      }
      fftw_destroy_plan(p);
   }

#ifdef TIME_REPORT
   t2=stoptime();
   printf("poisolve(): %5.0f ms: solution to normal space (fft)\n",(t2-t1)*1000.0);
   t1=t2;
#endif

   // by default, U only contains the inner grid points of the solution
   // however, we can also add the boundary if needed (inefficient)
   if(add_boundary_to_solution) {
      set_boundary(U,h1,h2,bd1a,bd1b,bd2a,bd2b,boundary,true);
   }

#ifdef TIME_REPORT
   t2=stoptime();
   printf("poisolve(): %5.0f ms: total time\n", (t2-t0)*1000.0);
#endif

   return sqrt(error);
}


// Poisson solver, assuming uniform boundary
double poisolve(boost::multi_array<double,2>& U,
                const boost::multi_array<double,2>& F,
                double a1, double a2, double h1, double h2,
                double bound_value,
                types::boundary boundarycondition,
                bool add_boundary_to_solution)
{

   size_t n1=F.shape()[0];
   size_t n2=F.shape()[1];

   // fill boundary vectors
   std::vector<double> bd1a,bd1b,bd2a,bd2b;
   set_boundary_to_uniform(bd1a,bd1b,bd2a,bd2b,bound_value,n1,n2);

   // call the general solver
   return poisolve(U,F,a1,a2,h1,h2,bd1a,bd1b,bd2a,bd2b,
                   boundarycondition,add_boundary_to_solution);
}

} // namespace pde


