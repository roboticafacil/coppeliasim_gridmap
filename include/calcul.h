/***************************************************/
/* Last Revised: 16/03/2020                        */
/***************************************************/

#ifndef Calcul
#define Calcul

#include <stdio.h>
#include <math.h>
#include "TData.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 
   This file has transformation operations of reference systems,
   point transformations between systems, convertion from polar coordinates,
   to Cartesian and segment cutting

*/

/* --------------------------------------------------------------------------------------- */
/* TRANSFORMACIONES DE PUNTO DE UN SISTEMA DE REFERENCIA A OTRO                            */
/* --------------------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------------------- */
/* direct_transform_p                                                                      */
/*  .... It does the direct transformation from one point to one system to another                  */
/*  .... In: (x,y) the coordinates of the point, system is the reference system         */
/*  .... Out: in "sol" the coordinates of the point are returned in the new system             */

void direct_transform_p ( double x, double y, Tsc *system, Tpf *sol );

/* --------------------------------------------------------------------------------------- */
/* direct_transform_p0                                                                      */
/*  .... It does the direct transformation from one point to one system to another                    */
/*  .... The difference is that here the entry point is (0,0) (optimize the previous one)  */
/*  .... In: (x,y) the coordinates of the point, system is the reference system         */
/*  .... Out: in "sol" the coordinates of the point are returned in the new system             */

void direct_transform_p0(double x, double y,
                          Tsc *system, Tpf *sol);
  
/* --------------------------------------------------------------------------------------- */
/* inverse_transform_p                                                                      */
/*  .... It does the inverse transformation from one point to one system to another                   */
/*  .... In: (x,y) the coordinates of the point, system is the reference system           */
/*  .... Out:  in "sol" the coordinates of the point are returned in the new system              */

void inverse_transform_p ( double x, double y, Tsc *system, Tpf *sol );

/* --------------------------------------------------------------------------------------- */
/* TRANSFORMACIONES DE COMPOSICION E INVERSION DE SISTEMAS DE REFERENCIA                   */
/* --------------------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------------------- */
/* system_composition                                                                         */
/*  .... Makes the composition of reference systems in another system                  */
/*  .... In: composes sys1 and sys2                                                           */
/*  .... Out: sisOut output is the result of the composition of the systems           */
/*  .... Nota: the order of the entries in the composition is very important           */

void system_composition(Tsc *sys1,Tsc *sys2,Tsc *sysOut);

/* --------------------------------------------------------------------------------------- */
/* system_inversion                                                                           */
/*  .... Inverts in a reference system                                  */
/*  .... In: sysIn reference system to invert                                               */
/*  .... Out: sysOut inverted reference system                                               */

void system_inversion(Tsc *sysIn, Tsc *sysOut);

/* --------------------------------------------------------------------------------------- */
/* TRANSFORMACIONES DE PUNTO DE UN SISTEMA DE REFERENCIA A OTRO                            */
/* --------------------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------------------- */
/* cart2pol                                                                                 */
/*  .... Transforms a point from Cartesian coordinates to polar coordinates                */
/*  .... In: Point in Cartesian coordinates                                                */
/*  .... Out: Point in polar coordinates                                                   */

void cart2pol(Tpf *in, Tpfp *out);

/* ---------------------------------------------------------------------------------------  */
/* pol2car t                                                                                */
/*  .... Transforms a point from polar coordinates to Cartesian coordinates                 */
/*  .... In: Point in polar coordinates                                                     */
/*  .... Out: Point in Cartesian coordinates                                                */

void pol2cart(Tpfp *in, Tpf *out);

/* --------------------------------------------------------------------------------------- */
/* TRANSFORMACIONES DE PUNTO DE UN SISTEMA DE REFERENCIA A OTRO                            */
/* --------------------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------------------- */
/* corte_segmentos                                                                         */ 
/*  .... Calcula el punto de corte entre dos segmentos                                     */ 
/*  .... In: las coordenadas de los puntos extremos (x1,y1)-(x2,y2) y (x3,y3)-(x4,y4)      */
/*  .... Out: sol son las coordenadas del punto de corte. return --> 1 si hay corte. -->0 no */

int segment_cut ( double x1, double y1, double x2, double y2,
                      double x3, double y3, double x4, double y4,
		      Tpf *sol );


/* Normalizes an angle between [-PI, PI] */
double NormalizePI(double ang);

#ifdef __cplusplus
}
#endif

#endif 
