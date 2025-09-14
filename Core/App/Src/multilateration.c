/*
 * multilateration.c
 *
 *  Created on: Sep 2, 2025
 *      Author: Marko Srpak
 */

/*--------------------------- INCLUDES ---------------------------------------*/
#include "multilateration.h"
#include "math.h"
#include <stdlib.h>
/*--------------------------- MACROS AND DEFINES -----------------------------*/
/*--------------------------- TYPEDEFS AND STRUCTS ---------------------------*/
/*--------------------------- STATIC FUNCTION PROTOTYPES ---------------------*/
/*--------------------------- VARIABLES --------------------------------------*/
/*--------------------------- STATIC FUNCTIONS -------------------------------*/
/*--------------------------- GLOBAL FUNCTIONS -------------------------------*/

void multilat_aprox_matrix(
    const coord_t anchors[4], // 4 anchora sa x,y,z
    const double distances[4],  // 4 udaljenosti s1,s2,s3,s4
    coord_t *est		  // izlaz: x,y,z estimacije
)
{
    int N = 4;
    double A[4][4];
    double b[4];
    double X[4]; // rješenje [c; x; y; z]

    // Konstrukcija matrice A i vektora b
    for (int i = 0; i < N; i++)
    {
        double xi = anchors[i].x;
        double yi = anchors[i].y;
        double zi = anchors[i].z;
        double si = distances[i];

        A[i][0] = 1.0;
        A[i][1] = -2.0 * xi;
        A[i][2] = -2.0 * yi;
        A[i][3] = -2.0 * zi;

        b[i] = si * si - xi * xi - yi * yi - zi * zi;
    }

    // Sustav 4 jednadzbe s 4 nepoznanice
    // Rješavanje sustava A*X = b metodom Gaussove eliminacije
    double M[4][5]; // augmentirana matrica [A|b]
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            M[i][j] = A[i][j];
        }
        M[i][4] = b[i];
    }

    // Gaussova eliminacija
    for (int i = 0; i < 4; i++)
    {
        // Max red za bolju numeričku stabilnost
        int max_row = i;
        for (int k = i + 1; k < 4; k++)
        {
            if (abs(M[k][i]) > abs(M[max_row][i]))
            {
                max_row = k;
            }
        }

        // Zamjena redova
        for (int j = 0; j < 5; j++)
        {
            double tmp = M[i][j];
            M[i][j] = M[max_row][j];
            M[max_row][j] = tmp;
        }

        // Eliminacija
        for (int k = i + 1; k < 4; k++)
        {
            double factor = M[k][i] / M[i][i];
            for (int j = i; j < 5; j++)
            {
                M[k][j] -= factor * M[i][j];
            }
        }
    }

    // Rješenje
    for (int i = 3; i >= 0; i--)
    {
        X[i] = M[i][4];
        for (int j = i + 1; j < 4; j++)
        {
            X[i] -= M[i][j] * X[j];
        }
        X[i] /= M[i][i];
    }

    // Rezultat: X = [c; x; y; z]
    est->x = X[1];
    est->y = X[2];
    est->z = X[3];
}

void multilat_gauss_iter_matrix(
    const coord_t anchors[4], // 4 anchora sa x,y,z
    const double distances[4],  // 4 udaljenosti s1,s2,s3,s4
    coord_t *est		  // izlaz: x,y,z estimacije
)
{
    int N = 4;
    int max_iter = 10;
    double tol = 1e-6;

    // --- start: linearna aproksimacija ---
    coord_t est0;
    multilat_aprox_matrix(anchors, distances, &est0);

    // fallback: centroid ako aproksimacija nije realna
    if (!(!isnan(est0.x) && !isnan(est0.y) && !isnan(est0.z)))
    {
        est0.x = 0.0;
        est0.y = 0.0;
        est0.z = 0.0;
        for (int i = 0; i < N; i++)
        {
            est0.x += anchors[i].x;
            est0.y += anchors[i].y;
            est0.z += anchors[i].z;
        }
        est0.x /= N;
        est0.y /= N;
        est0.z /= N;
    }

    // --- iterativni Gauss-Newton ---
    for (int iter = 0; iter < max_iter; iter++)
    {
        double r[4];    // reziduali
        double J[4][3]; // Jacobian

        for (int i = 0; i < N; i++)
        {
            double dx = est0.x - anchors[i].x;
            double dy = est0.y - anchors[i].y;
            double dz = est0.z - anchors[i].z;
            double di = sqrt(dx * dx + dy * dy + dz * dz);
            r[i] = di - distances[i];

            if (di < 1e-12)
            {
                J[i][0] = J[i][1] = J[i][2] = 0.0;
            }
            else
            {
                J[i][0] = dx / di;
                J[i][1] = dy / di;
                J[i][2] = dz / di;
            }
        }

        // --- Gauss-Newton korak: delta = -(J' * J)^-1 * J' * r ---
        // Za 4x3 Jacobian i 4x1 r koristimo normalne jednadžbe:
        // delta = - (J^T J)^(-1) J^T r
        double JTJ[3][3] = {0};
        double JTr[3] = {0};
        for (int i = 0; i < N; i++)
        {
            for (int m = 0; m < 3; m++)
            {
                JTr[m] += J[i][m] * r[i];
                for (int n = 0; n < 3; n++)
                {
                    JTJ[m][n] += J[i][m] * J[i][n];
                }
            }
        }

        // --- invertiranje 3x3 matrice JTJ ---
        double a = JTJ[0][0], b = JTJ[0][1], c = JTJ[0][2];
        double d = JTJ[1][0], e = JTJ[1][1], f = JTJ[1][2];
        double g = JTJ[2][0], h = JTJ[2][1], i3 = JTJ[2][2];

        double det = a * (e * i3 - f * h) - b * (d * i3 - f * g) + c * (d * h - e * g);
        if (fabs(det) < 1e-12)
            break; // singularni slučaj

        double invJTJ[3][3];
        invJTJ[0][0] = (e * i3 - f * h) / det;
        invJTJ[0][1] = -(b * i3 - c * h) / det;
        invJTJ[0][2] = (b * f - c * e) / det;
        invJTJ[1][0] = -(d * i3 - f * g) / det;
        invJTJ[1][1] = (a * i3 - c * g) / det;
        invJTJ[1][2] = -(a * f - c * d) / det;
        invJTJ[2][0] = (d * h - e * g) / det;
        invJTJ[2][1] = -(a * h - b * g) / det;
        invJTJ[2][2] = (a * e - b * d) / det;

        // delta = -inv(JTJ) * JTr
        double delta[3];
        for (int m = 0; m < 3; m++)
        {
            delta[m] = 0.0;
            for (int n = 0; n < 3; n++)
            {
                delta[m] += invJTJ[m][n] * JTr[n];
            }
            delta[m] = -delta[m];
        }

        // update
        est0.x += delta[0];
        est0.y += delta[1];
        est0.z += delta[2];

        // stop kriterij
        double norm_delta = sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
        if (norm_delta < tol)
            break;
    }

    est->x = est0.x;
    est->y = est0.y;
    est->z = est0.z;
}
