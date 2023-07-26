#pragma once
#include "plfh_solver.h"
#include <random>
#include <Eigen/Dense>
using namespace Eigen;

#define R_PLANE_NUMBER 4

void BilateralConsensus(vector<pcl::PLFH_gather> plfh_set_S, vector<pcl::PLFH_gather> plfh_set_T);