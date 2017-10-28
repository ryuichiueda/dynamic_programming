#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

// FUNCTIONS FOR SETUP
int numberCells(float, float *);
void setVector(int, float, float *, float *);
void setObst(float *);
void setGoal(float *, float *, float *);
void setInitialValue(float *, float *, float *, int, int, int);
void setInitialPolicy(float *, float *, char *);

// FUNCTIONS FOR VALUE ITERATION
void valueIteration(float *, float *, float *, char *, float *);
void conditionR(int, int, int, float *, float *);
void conditionTheta(int, int, int, float *, float *);
void conditionPhi(int, int, int, float *, float *);
void computeTotalCost(float *, float *);
float computeNewValue(float *);
char computeNewPolicy(float *);

// DEFINE GLOBAL PARAMETERS
int nr, ntheta, nphi;
float perr;
float gamma1;
float vGoal, vObst, vMove;
float vInitial;
int numActions = 7;

int main(int argc, char **argv) 
{
	// DEFINE PARAMETERS
	float dr, dtheta, dphi;
	float rdim[2], thetadim[2], phidim[2];
	float *rVec, *thetaVec, *phiVec;
	
	// - minimum grid resolution for r, theta, phi
	dr = 5.0;
	dtheta = 180.0;
	dphi = 180.0;
	// - dimensions of the state space
	rdim[0] = 0.0, rdim[1] = 10.0;
	thetadim[0] = 0.0, thetadim[1] = 360.0;
	phidim[0] = 0.0, phidim[1] = 360.0;
	// - number of grid cells
	nr = numberCells(dr, rdim);
	ntheta = numberCells(dtheta, thetadim);
	nphi = numberCells(dphi, phidim);
	// - vectors for r, theta, phi
	rVec = (float *)malloc(sizeof(float)*nr);
	thetaVec = (float *)malloc(sizeof(float)*ntheta);
	phiVec = (float *)malloc(sizeof(float)*nphi);
	setVector(nr, dr, rdim, rVec);
	setVector(ntheta, dtheta, thetadim, thetaVec);
	setVector(nphi, dphi, phidim, phiVec);
	// - probability of going the wrong way
	perr = 0.0;
	// attenuation rate
	gamma1 = 1.0;
	// - value of goal, collision, movement
	vGoal = 100.0;
	vObst = -100.0;
	vMove = -1.0;
	// initial guess at all values
	vInitial = 0.0; 

	// DEFINE OBSTACLE AND GOAL LOCATIONS
	float *isobst, *isgoal;
	isobst = (float *)calloc(nr*ntheta*nphi, sizeof(float));
	isgoal = (float *)calloc(nr*ntheta*nphi, sizeof(float));
	setObst(isobst);
	setGoal(thetaVec, phiVec, isgoal);

	// DEFINE INITIAL GUESS AT VALUE AND POLICY
	float *J;
	char *U;
	J = (float *)calloc(nr*ntheta*nphi, sizeof(float));
	U = (char *)calloc(nr*ntheta*nphi, sizeof(char));
	setInitialValue(isobst, isgoal, J, vObst, vGoal, vInitial);
	setInitialPolicy(isobst, isgoal, U);

	// DO VALUE ITERATION
	int T = 100;
	float *Jprev;
	char *Uprev;
	Jprev = (float *)calloc(nr*ntheta*nphi, sizeof(float));
	Uprev = (char *)calloc(nr*ntheta*nphi, sizeof(char));

	for(int t=0; t<T; t++)
	{
		printf("Iteration %d\n", t+1);

		// Iterate over all states.
		memcpy(Jprev, J, sizeof(float)*nr*ntheta*nphi);
		memcpy(Uprev, U, sizeof(char)*nr*ntheta*nphi);

		valueIteration(isobst, isgoal, J, U, Jprev);

		for(int x=0; x<nr*ntheta*nphi; x++)
		{
			printf("%2d J=%3.1f U=%2d\n", x, J[x], U[x]);
		}
		printf("\n");
	}

	// free used memory
	free(rVec);
	free(thetaVec);
	free(phiVec);
	free(isobst);
	free(isgoal);
	free(J);
	free(U);
	free(Jprev);
	free(Uprev);
	
	return(0);
}

/*--------------- FUNCTIONS ----------------*/

int numberCells(float d, float *dim)
{
	int n = 0;
	float diff;
	diff = dim[1]-dim[0];

	if(d<1 || d>diff)
	{
		printf("value of resolution or dimension is invalid.\n");
	}
	else
	{
		n = floorf(diff/d+1.0);
	}

	return n;
}

void setVector(int n, float d, float *dim, float *Vec)
{
	float value;
	value = dim[0];

	for(int i=0; i<n; i++)
	{
		Vec[i] = value;
		value += d;
	}
}

void setObst(float *isobst)
{
	for(int j=0; j<ntheta; j++)
	{
		for(int k=0;k<nphi; k++)
		{
			isobst[nr*ntheta*k+(nr-1)*ntheta+j] = 1;
		}
	}
}

void setGoal(float *thetaVec, float *phiVec, float *isgoal)
{
	for(int j=0; j<ntheta; j++)
	{
		for(int k=0; k<nphi; k++)
		{
			if(thetaVec[j]==phiVec[k])
				isgoal[nr*ntheta*k+j] = 1;
		}
	}
}

void setInitialValue(float *isobst, float *isgoal, float *J, int vObst, int vGoal, int vInitial)
{
	for(int i=0; i<nr; i++)
	{
		for(int j=0; j<ntheta; j++)
		{
			for(int k=0; k<nphi; k++)
			{
				if(isobst[nr*ntheta*k+ntheta*i+j])
				{
					J[nr*ntheta*k+ntheta*i+j] = vObst;
				}
				else if(isgoal[nr*ntheta*k+ntheta*i+j])
				{
					J[nr*ntheta*k+ntheta*i+j] = vGoal;
				}
				else
				{
					J[nr*ntheta*k+ntheta*i+j] = vInitial;
				}
			}
		}
	}
}

void setInitialPolicy(float *isobst, float *isgoal, char *U)
{
	srand((unsigned)time(NULL));

	for(int i=0; i<nr; i++)
	{
		for(int j=0; j<ntheta; j++)
		{
			for(int k=0; k<nphi; k++)
			{
				if(isobst[nr*ntheta*k+ntheta*i+j])
				{
					U[nr*ntheta*k+ntheta*i+j] = -1;
				}
				else if(isgoal[nr*ntheta*k+ntheta*i+j])
				{
					U[nr*ntheta*k+ntheta*i+j] = -1;
				}
				else
				{
					char r = rand() % 7;
					U[nr*ntheta*k+ntheta*i+j] = r;
				}
			}
		}
	}
}

void valueIteration(float *isobst, float *isgoal, float *J, char *U, float *Jprev)
{
	float *tempCost, *totalCost;
	tempCost = (float *)calloc(numActions, sizeof(float));
	totalCost = (float *)calloc(numActions, sizeof(float));

	for(int i=0; i<nr; i++)
	{
		for(int j=0; j<ntheta; j++)
		{
			for(int k=0; k<nphi; k++)
			{
				if(!isobst[nr*ntheta*k+ntheta*i+j] && !isgoal[nr*ntheta*k+ntheta*i+j])
				{
					tempCost[0]=Jprev[nr*ntheta*k+ntheta*i+j];
					// condition of r
					conditionR(i, j, k, tempCost, Jprev);

					// Compute the total expected cost for each of the possible actions.
					computeTotalCost(tempCost, totalCost);

					// Compute the new exptected cost-to-go, by taking the maximum over
					// possible actions.
					J[nr*ntheta*k+ntheta*i+j] = computeNewValue(totalCost);
					U[nr*ntheta*k+ntheta*i+j] = computeNewPolicy(totalCost);

				}
			}
		}
	}

	free(tempCost);
	free(totalCost);
}

void conditionR(int i, int j, int k, float *tempCost, float *Jprev)
{
	if(i==0)
	{
		tempCost[1] = Jprev[nr*ntheta*k+ntheta*(i+1)+j];
		tempCost[2] = Jprev[nr*ntheta*k+ntheta*i+j];
		conditionTheta(i, j, k, tempCost, Jprev);
	}
	else
	{
		tempCost[1] = Jprev[nr*ntheta*k+ntheta*(i+1)+j];
		tempCost[2] = Jprev[nr*ntheta*k+ntheta*(i-1)+j];
		conditionTheta(i, j, k, tempCost, Jprev);
	}
}

void conditionTheta(int i, int j, int k, float *tempCost, float *Jprev)
{
	if(j==0)
	{
		tempCost[3] = Jprev[nr*ntheta*k+ntheta*i+(j+1)];
		tempCost[4] = Jprev[nr*ntheta*k+ntheta*i+(ntheta-1)];
		conditionPhi(i, j, k, tempCost, Jprev);
	}
	else if(j==ntheta-1)
	{
		tempCost[3] = Jprev[nr*ntheta*k+ntheta*i];
		tempCost[4] = Jprev[nr*ntheta*k+ntheta*i+(j-1)];
		conditionPhi(i, j, k, tempCost, Jprev);
	}
	else
	{
		tempCost[3] = Jprev[nr*ntheta*k+ntheta*i+(j+1)];
		tempCost[4] = Jprev[nr*ntheta*k+ntheta*i+(j-1)];
		conditionPhi(i, j, k, tempCost, Jprev);
	}
}

void conditionPhi(int i, int j, int k, float *tempCost, float *Jprev)
{
	if(k==0)
	{
		tempCost[5] = Jprev[nr*ntheta*(k+1)+ntheta*i+j];
		tempCost[6] = Jprev[nr*ntheta*(nphi-1)+ntheta*i+j];
	}
	else if(k==nphi-1)
	{
		tempCost[5] = Jprev[ntheta*i+j];
		tempCost[6] = Jprev[nr*ntheta*(k-1)+ntheta*i+j];
	}
	else
	{
		tempCost[5] = Jprev[nr*ntheta*(k+1)+ntheta*i+j];
		tempCost[6] = Jprev[nr*ntheta*(k-1)+ntheta*i+j];
	}
}

void computeTotalCost(float *tempCost, float *totalCost)
{
	totalCost[0] = vMove+gamma1*((1-perr)*tempCost[0]+(perr/6)*(tempCost[1]+tempCost[2]+tempCost[3]+tempCost[4]+tempCost[5]+tempCost[6]));
	totalCost[1] = vMove+gamma1*((1-perr)*tempCost[1]+(perr/6)*(tempCost[0]+tempCost[2]+tempCost[3]+tempCost[4]+tempCost[5]+tempCost[6]));
	totalCost[2] = vMove+gamma1*((1-perr)*tempCost[2]+(perr/6)*(tempCost[1]+tempCost[0]+tempCost[3]+tempCost[4]+tempCost[5]+tempCost[6]));
	totalCost[3] = vMove+gamma1*((1-perr)*tempCost[3]+(perr/6)*(tempCost[1]+tempCost[2]+tempCost[0]+tempCost[4]+tempCost[5]+tempCost[6]));
	totalCost[4] = vMove+gamma1*((1-perr)*tempCost[4]+(perr/6)*(tempCost[1]+tempCost[2]+tempCost[3]+tempCost[0]+tempCost[5]+tempCost[6]));
	totalCost[5] = vMove+gamma1*((1-perr)*tempCost[5]+(perr/6)*(tempCost[1]+tempCost[2]+tempCost[3]+tempCost[4]+tempCost[0]+tempCost[6]));
	totalCost[6] = vMove+gamma1*((1-perr)*tempCost[6]+(perr/6)*(tempCost[1]+tempCost[2]+tempCost[3]+tempCost[4]+tempCost[5]+tempCost[0]));
}

float computeNewValue(float *totalCost)
{
	float max;
	max = totalCost[0];

	for(int n=0; n<numActions; n++)
	{
		if(totalCost[n]>max)
			max=totalCost[n];
	}

	return max;
}

char computeNewPolicy(float *totalCost)
{
	float max;
	char idx;
	max = totalCost[0];

	for(int n=0; n<numActions; n++)
	{
		if(totalCost[n]>max)
		{
			max=totalCost[n];
			idx=n;
		}

	}

	return idx;
}