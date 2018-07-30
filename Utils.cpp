#include "Utils.h"
#include "Defs.h"

using namespace std;

static double getDist(double x1, double y1, double z1, double x2, double y2, double z2) {
	double x = x1 - x2;
	double y = y1 - y2;
	double z = z1 - z2;
	double squared_dist = x*x + y*y + z*z;
	return sqrt(squared_dist);
}

double Utils::GetTotalAngle(const ob::State *s1, const ob::State *s2) {
	const auto *pos1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
	const auto *angle1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
	const auto *pos2 = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
	const auto *angle2 = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

	return this->GetTotalAngle(pos1->values[0], pos1->values[1], angle1->values[0], pos2->values[0], pos2->values[1], angle2->values[0]);
}

double Utils::GetTotalAngle(double x1, double y1, double theta1, double x2, double y2, double theta2) {
	double phi; 
	double a1;
	double a2;
	double val;

	phi = atan2(y2 - y1, x2 - x1); // getting the angle in PI units
	phi = ( phi >= 0 ? phi : ( 2*PI + phi ) ) * 360 / ( 2*PI );
	//printf("PHI: %.2f\n",phi);
	val = std::abs(theta1 - phi);
	a1 = (val < 180) ? val : 360 - val;
	val = std::abs(theta2 - phi);
	a2 = (val < 180) ? val : 360 - val;

	return (a1 + a2);
}

double Utils::GetEuclidDistance(const ob::State *s1, const ob::State *s2) {
	const auto *pos1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
	const auto *pos2 = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
	return getDist(pos1->values[0], pos1->values[1], pos1->values[2], pos2->values[0], pos2->values[1], pos2->values[2]);
}

double Utils::GetEuclidDistance(double x1, double y1, double z1, double x2, double y2, double z2) {
	return getDist(x1, y1, z1, x2, y2, z2);
}