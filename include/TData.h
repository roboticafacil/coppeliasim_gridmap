/***************************************************/
/* Last Revised: 
$Id: TData.h 4129 2007-08-21 23:16:24Z gerkey $
*/
/***************************************************/

#ifndef TData
#define TData

#ifdef __cplusplus
extern "C" {
#endif

/* 
   Este fichero contiene los tipos de datos utilizados por todos 
*/

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAXLASERPOINTS 1000

#define RADIO 0.4F  /* Radio del robot */

typedef struct {
  double x;
  double y;
}Tpf;


typedef struct {
  double r;
  double t;
}Tpfp;

typedef struct {
  int x;
  int y;
}Tpi;

typedef struct {
  double x;
  double y;
  double theta;
}Tsc;

typedef struct {
  int numPoints;
  Tpf laserC[MAXLASERPOINTS];  // Cartesian coordinates
  Tpfp laserP[MAXLASERPOINTS]; // Polar coordinates
}Tscan;


// Associations information
typedef struct{
  double rx,ry,nx,ny,dist;				// Point (nx,ny), static corr (rx,ry), dist
  int L,R;
  double lambda;
  double q1x,q1y,q2x,q2y;
}TAsoc;

#ifdef __cplusplus
}
#endif

#endif
