#define _CRT_SECURE_NO_DEPRECATE 1
#include <iostream>
#include <unistd.h>
#include <ctype.h>
#include <vector>
#include <cmath>
#include "command.h"
#include "kalmanAgent.h"
#include "randomAgent.h"

using namespace std;

//cmd: ./bin/bzrflag --world=maps/blank.bzw --blue-tanks=1 --red-tanks=1 --green-tanks=1 --purple-tanks=1 --default-posnoise=5

int myX = -390;
int myY = 0;


double myAngle(BZRC* command){
    vector<tank_t> tanks;
    command->get_mytanks(tanks);
    double a = tanks[0].angle;
    if(a>=0){
        cout << "me "  << a << endl;
        return a;
    }else{
        cout << "me "  << 6.28+a << endl;
        return 6.28 + a;
    }
}

double myAngle(double a){
    if(a>=0){
        return a;
    }else{
        return 6.28 + a;
    }
}


double theirAngle(Node n){
    double ret = myAngle(atan2(-myY + n.y, -myX + n.x));
    cout << "them: " <<ret << endl;
    // return myAngle(atan2(myY - n.y, myX - n.x));
    return ret;
}

double changeInAngle(BZRC* command, Node n){
    double me = myAngle(command);
    double them = theirAngle(n);
    double large = me > them ? me : them;
    double small = me > them ? them : me;
    double diff = large- small;


    diff = me > them ? 6.28-diff : diff; 

    return diff;
}


int main(int argc, char *argv[]) {
	const char *pcHost = "127.0.0.1";
	int portpurple = 0;
	int portgreen = 0;
	int portblue = 0;
	int portred = 0;
	int option;

	while ((option = getopt(argc,argv,"s:p:g:b:r:")) != -1) {
        switch (option) {
            case 'p':
                portpurple = atoi(optarg);
                break;
            case 'g':
            	portgreen = atoi(optarg);
            	break;
            case 'b':
            	portblue = atoi(optarg);
            	break;
            case 'r':
            	portred = atoi(optarg);
            	break;	
            case 's':
                pcHost = optarg;
                break;
            default:
                cout << "client [-s IP address] [-c port], where c is the first char of the color" << endl;
                exit(EXIT_FAILURE);
        }
    }
    BZRC* blue = NULL;
    BZRC* red = NULL;
    BZRC* purple = NULL;
    BZRC* green = NULL;
    if(portred)
    	red = new BZRC(pcHost, portred, false);
    if(portpurple)
    	purple = new BZRC(pcHost, portpurple, false);
    if(portblue)
    	blue = new BZRC(pcHost, portblue, false);
    if(portgreen)
    	green = new BZRC(pcHost, portgreen, false);

    green->speed(0,0.3);

    double sleepAmount = 1;    

    RandomAgent random(0, blue, 10);
    KalmanAgent kalmanGreen(0, purple, sleepAmount, .0, "blue");
    // KalmanAgent kalmanBlue(0, purple, sleepAmount, .0, "blue");
    // KalmanAgent kalmanRed(0, purple, sleepAmount, .0, "red");

    ofstream greenFile;
    ofstream redFile;
    ofstream blueFile;
    greenFile.open ("values-green.dat");
    redFile.open ("values-red.dat");
    blueFile.open ("values-blue.dat");

    while(true){
        bool stopped = false;
        purple->speed(0,0);
        for (int i = 0; i < 20; i++){
            sleep(sleepAmount);
            random.move();
            greenFile << kalmanGreen.update();
            Node n = kalmanGreen.getPos();
            double a = changeInAngle(purple,n);
            cout << a << endl;
            if(a>0.8 && !stopped){
                purple->angvel(0,0.8);
            }else{
                purple->speed(0,1);
                stopped = true;
                purple->angvel(0,0);
            }
        }
        purple->angvel(0,0);
        purple->speed(0,0);

        greenFile << endl;
        stopped = false;


        greenFile << kalmanGreen.predict(5*(sleepAmount));
        Node futurePos = kalmanGreen.getPos();
        double futureA = changeInAngle(purple,futurePos);

        purple->angvel(0,1);
        cout << "futureA " <<  futureA << endl;
        double timeleft = 5 - abs(futureA/0.601); 
        cout << "sleep time " << max(abs(futureA/0.601)-0.05,0.0) << endl;
        sleep(max(abs(futureA/0.601)-0.05,1.0));
        purple->angvel(0,0);
        double dist = sqrt(pow(futurePos.x-myX,2)+pow(futurePos.y-myY,2));
        double bullettime = dist/100;
        double sleeptime = timeleft - bullettime;
        if(sleeptime>0){
            sleep(sleeptime);
        }
        purple->shoot(0);

        kalmanGreen.reset();
        
    }

    greenFile.close();
    redFile.close();
    blueFile.close();

    if(purple)
        delete purple;
    if(red)
        delete red;
    if(blue)
        delete blue;
    if(green)
        delete green;

	return 0;
}