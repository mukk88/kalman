#ifndef KALMANAGENT_H
#define KALMANAGENT_H
#include <sstream>
#include "command.h"
#include <algorithm>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <string>
#include <iterator>
#include <fstream>
#include <iostream>

using Eigen::MatrixXd;



class KalmanAgent {

public:

	KalmanAgent(int i, BZRC* bzrc, double changeInT, double friction, string c) 
	: index(i), commandCenter(bzrc), color(c), f(friction) {
		int x, y;
		x = y = 0;
		if (color.compare("blue") == 0){
			x = 150;
			y = -100;
		}
		else if (color.compare("red") == 0){
			x = -150;
			y = -100;
		}
		else {
			x = 0;
			y = -100;
		}
		MatrixXd u2(6,1);
		u2 << x, 0, 0, y, 0, 0;
		u = u2;
		
		F = getForceMatrix(changeInT, 1);
		Ftranspose = F.transpose();
		reset();
	}

	void reset(){
		// MatrixXd Sigmat2(6,6);
		// Sigmat2 << 5, 0, 0, 0, 0, 0, 
		// 	0, .01, 0, 0, 0, 0, 
		// 	0 , 0, .01, 0, 0, 0, 
		// 	0, 0, 0, 5, 0, 0, 
		// 	0, 0, 0, 0, .01, 0, 
		// 	0, 0, 0, 0, 0, .01;
		// Sigmat = Sigmat2;
		// MatrixXd Sigmax2(6,6);
		// Sigmax2 << .1, 0, 0, 0, 0, 0, 
		// 	0, .02, 0, 0, 0, 0, 
		// 	0 , 0, .01, 0, 0, 0, 
		// 	0, 0, 0, .1, 0, 0, 
		// 	0, 0, 0, 0, .02, 0, 
		// 	0, 0, 0, 0, 0, .01;
		// Sigmax = Sigmax2;
		// MatrixXd H2(2, 6);
		// H2 << 1, 0, 0, 0, 0, 0, 
		// 	0, 0, 0, 1, 0, 0;
		// H = H2;
		// Htranspose = H.transpose();
		// MatrixXd Sigmaz2(2, 2);
		// Sigmaz2 << 25, 0, 0, 25;
		// Sigmaz = Sigmaz2;
		MatrixXd Sigmat2(6,6);
		Sigmat2 << 100, 0, 0, 0, 0, 0, 
			0, .1, 0, 0, 0, 0, 
			0 , 0, .1, 0, 0, 0, 
			0, 0, 0, 100, 0, 0, 
			0, 0, 0, 0, .1, 0, 
			0, 0, 0, 0, 0, .1;
		Sigmat = Sigmat2;
		MatrixXd Sigmax2(6,6);
		Sigmax2 << .1, 0, 0, 0, 0, 0, 
			0, .1, 0, 0, 0, 0, 
			0 , 0, 100, 0, 0, 0, 
			0, 0, 0, .1, 0, 0, 
			0, 0, 0, 0, .1, 0, 
			0, 0, 0, 0, 0, 100;
		Sigmax = Sigmax2;
		MatrixXd H2(2, 6);
		H2 << 1, 0, 0, 0, 0, 0, 
			0, 0, 0, 1, 0, 0;
		H = H2;
		Htranspose = H.transpose();
		MatrixXd Sigmaz2(2, 2);
		Sigmaz2 << 25, 0, 0, 25;
		Sigmaz = Sigmaz2;
	}

	MatrixXd GetKtplus1(MatrixXd force){
		//(FΣtFT + Σx)HT(H(FΣtFT + Σx)HT + Σz) − 1
		MatrixXd temp = (force*Sigmat*force.transpose() + Sigmax);
		MatrixXd test = temp * Htranspose * 
			(H * temp * Htranspose + Sigmaz).inverse();
		// std::cout << Sigmat << std::endl << "\n";
		return test;
	}

	MatrixXd getObservation(){
		double x, y;
		std::vector <otank_t> otherTanks;
		commandCenter->get_othertanks(&otherTanks);
		for (int i = 0; i < otherTanks.size(); ++i){
			if (color.compare(otherTanks[i].color) == 0){
				x = otherTanks[i].pos[0];
				y = otherTanks[i].pos[1];
				break;
			}
		}
		MatrixXd z(2, 1);
		z << x, y;
		return z;
	}

	string update(double changeInTime){
		MatrixXd z = getObservation();
		MatrixXd F2 = getForceMatrix(changeInTime, 0);
		MatrixXd Ktplus1 = GetKtplus1(F2);
		 
		updateMean(Ktplus1, z, changeInTime);
		updateError(Ktplus1, changeInTime);
		std::cout << Sigmat(0,0)  << " " <<  Sigmat(3,3) << std::endl;
		//std::cout << z << std::endl << std::endl;

		return outputValues(u);
	}

	string outputValues(MatrixXd temp){
		//row sigmax sigmay x y
		double sigmax = Sigmat(0,0);
		double sigmay = Sigmat(3,3);
		double meanx = temp(0,0);
		double meany = temp(3,0);
		double accx = temp(2,0);
		double accy = temp(5,0);
		double velx = temp(1,0);
		double vely = temp(4,0);
		MatrixXd z = getObservation();
		stringstream ss;
		// ss << 0 << " " << sigmax << " " << sigmay << " " << 
		// 	meanx << " " << meany << " " << velx << " " << vely << " " << accx << " " << accy << "\n";
		ss << meanx << " " << meany << " " << z(0,0) << " "  << z(1,0) << "\n";
		return ss.str();
	}

	string predict(int changeTime){

		f = 0;
		// std::cout << u << std::endl;
		MatrixXd F2 = getForceMatrix(changeTime, 0);

		MatrixXd z = getObservation();
		// std::cout << z << std::endl << std::endl;

		// MatrixXd Ktplus1 = GetKtplus1(F2);
		MatrixXd u2 = F2*u;
		// updateError(Ktplus1);
		return outputValues(u2);
	}

	void updateMean(MatrixXd Ktplus1, MatrixXd z, double changeInTime){
		//μt + 1 = Fμt + Kt + 1(zt + 1 − HFμt)
		MatrixXd F2 = getForceMatrix(changeInTime, 1);
		u = F2*u + Ktplus1*(z - H*F2*u);
	}

	void updateError(MatrixXd Ktplus1, double changeInTime){
		//Σt + 1 = (I − Kt + 1H)(FΣtFT + Σx) 
		MatrixXd F2 = getForceMatrix(changeInTime, 1);
		Sigmat = (Eigen::Matrix<double, 6, 6>::Identity() - 
			Ktplus1*H)*(F2*Sigmat*F2.transpose() + Sigmax);
	}

	MatrixXd getForceMatrix(double changeInT, int acc){
		MatrixXd F2(6,6);
		acc = 1;
		f = 0;
		F2 << 1, changeInT, acc*(pow(changeInT, 2) / 2), 0, 0, 0,
			0, 1, acc*changeInT, 0, 0, 0,
			0, (-1 * f), acc*1, 0, 0, 0,
			0, 0, 0, 1, changeInT, acc*(pow(changeInT, 2) / 2),
			0, 0, 0, 0, 1, acc*changeInT,
			0, 0, 0, 0, (-1 * f), acc*1;
		return F2;
	}

	Node getPos(){
		Node n;
		n.x = u(0,0);
		n.y = u(3,0);
		return n;
	}



private:
	double f;
	string color;
	int index;
	BZRC* commandCenter;
	MatrixXd Ftranspose;
	MatrixXd Htranspose;
	MatrixXd F;
	MatrixXd Sigmat;
	MatrixXd Sigmax;
	MatrixXd Sigmaz;
	MatrixXd H;
	MatrixXd u;
};

#endif