#include "konami.h"

/* this code needs fpu enabled (-mfpu=vfpv3) */

#define TAP COEF_COUNT
#define NB TAP		// desired tapsize
#define NG NB		// invert tapsize
#define M_D NG/2  // emulate delay (as many as half the tap #)

void Gauss_invert(double a[NG][NG], double b[NG][NG], int n);

int calc_filter_lsq(double *impulse, double *result)
{
	int i,j,k;

	double C1[TAP+NG-1];
	double H[TAP+NG-1][NG];
	double H_t[NG][TAP+NG-1];
	double H_tH[NG][NG];
	double iH_tH[NG][NG];
	double iH_tHH_t[NG][TAP+NG-1];

	memset(C1, 0, sizeof(double) * TAP+NG-1);
	memset(H, 0, sizeof(double) * TAP+NG-1 * NG);
	memset(H_t, 0, sizeof(double) * TAP+NG-1 * NG);
	memset(iH_tHH_t, 0, sizeof(double) * TAP+NG-1 * NG);
	memset(H_tH, 0, sizeof(double) * NG * NG);
	memset(iH_tH, 0, sizeof(double) * NG * NG);

	for (i = 0; i < NG; i++) {
		for (k = 0; k < TAP; k++) {
			H[k+i][i] = impulse[k];
			H_t[i][k+i] = impulse[k];
		}
	}

	for (i = 0; i < NG; i++) {
		for (j = 0; j < NG; j++) {
			for (k = 0; k < TAP+NG-1; k++) {
				H_tH[i][j] += H_t[i][k] * H[k][j];
			}
		}
	}

	Gauss_invert(H_tH, iH_tH, NG);
	
	for (i = 0; i < NG; i++) {
		for (j = 0; j < TAP+NG-1; j++) {
			for (k = 0; k < NG; k++) {
				iH_tHH_t[i][j] += iH_tH[i][k] * H_t[k][j];
			}
		}
	}

	for(i = 0; i < NG; i++) {
		result[i] = iH_tHH_t[i][M_D];
	}

	return 0;
}

void Gauss_invert(double a[NG][NG], double b[NG][NG], int n)
{
	double d, temp = 0, c;
	int i, j, k, m, nn, *ipvt;

	if ((ipvt = (int *) malloc(n * sizeof(int))) == NULL)
		return;

	nn = n;
	for (i = 0; i < nn; i++)
		ipvt[i] = i;
	
	for (k = 0; k < nn; k++) {
		temp == 0.;
		m = k;
		for (i = k; i < nn; i++) {
			d = a[k][i];
			if (fabs(d) > temp) {
				temp = fabs(d);
				m = i;
			}
		}
		if (m != k) {
			j = ipvt[k];
			ipvt[k] = ipvt[m];
			ipvt[m] = j;
			for (j = 0; j < nn; j++) {
				temp = a[j][k];
				a[j][k] = a[j][m];
				a[j][m] = temp;
			}
		}
		d = 1/ a[k][k];
		for (j = 0; j < k; j++) {
			c = a[j][k] * d;
			for (i = 0; i < nn; i++)
				a[j][i] -= a[k][i] * c;
			a[j][k] = c;
		}
		for (j = k + 1; j < nn; j++) {
			c = a[j][k] * d;
			for (i = 0; i < nn; i++)
				a[j][i] -= a[k][i] * c;
			a[j][k] = c;
		}
		for (i = 0; i < nn; i++)
			a[k][i] = -a[k][i] * d;
		a[k][k] = d;
	}

	for (i = 0; i < nn; i++)
		memcpy(b[ipvt[i]], a[i], sizeof(double) * nn);
	
	free(ipvt);
}
