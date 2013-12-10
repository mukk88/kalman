#define _CRT_SECURE_NO_DEPRECATE 1
#define _USE_MATH_DEFINES
#include <iostream>
#include <unistd.h>
#include <ctype.h>
#include <vector>
#include <cmath>
#include "command.h"
#include "kalmanAgent.h"
#include "randomAgent.h"
#include <time.h>

using namespace std;

//cmd: ./bin/bzrflag --world=maps/blank.bzw --blue-tanks=1 --red-tanks=1 --green-tanks=1 --purple-tanks=1 --default-posnoise=5

double myX = 0;
double myY = 150;


double myAngle(BZRC* command){
    vector<tank_t> tanks;
    command->get_mytanks(tanks);
    double a = tanks[0].angle;
    if(a>=0){
        //cout << "me "  << a << endl;
        return a;
    }else{
        //cout << "me "  << 6.28+a << endl;
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
    return ret;
}

double changeInAngle(BZRC* command, Node n){

    double me = myAngle(command);
    double them = theirAngle(n);
    double diff = me - them;
    if (abs(diff) <= M_PI)
        return -diff;
    else {
        if (diff < 0){ // negative
            return -1*(M_PI*2 + diff);
        }
        else {
            return -1 * (diff - 2*M_PI);
        }
    }
}

double getBulletTime(BZRC* purple, Node futurePos){
    vector<tank_t> tanks;
    purple->get_mytanks(tanks);
    myX = tanks[0].pos[0];
    myY = tanks[0].pos[1];
    double dist = sqrt(pow(futurePos.x-myX,2)+pow(futurePos.y-myY,2));
    return dist/100;
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

    double sleepAmount = .5;    

    RandomAgent random(0, blue, 5);
    random.move();
    KalmanAgent kalmanGreen(0, purple, sleepAmount, 0, "green");
    KalmanAgent kalmanBlue(0, purple, sleepAmount, 0, "blue");
    KalmanAgent kalmanRed(0, purple, sleepAmount, 0, "red");

    ofstream greenFile;
    ofstream redFile;
    ofstream blueFile;
    greenFile.open ("values-green.dat");
    redFile.open ("values-red.dat");
    blueFile.open ("values-blue.dat");

    bool shoot = false;
    time_t currentTime;
    time_t prevTime;
    time(&prevTime);
    while (true){
        bool stopped = false;
        purple->speed(0,0);



        random.move();
        greenFile << kalmanGreen.update(sleepAmount);
        blueFile << kalmanBlue.update(sleepAmount);
        redFile << kalmanRed.update(sleepAmount);
        Node n = kalmanGreen.getPos();
        cout << "enemy tank: " << n.x << " " << n.y << endl;
        cout << "angle to enemy: " << changeInAngle(purple,n) << endl;



        //while (abs(changeInAngle(purple,n)) > .005){
        for (int i = 0; i < 15; ++i) {
            if (shoot)
                usleep(10000);//sleepAmount);
            else 
                usleep(sleepAmount * 1000000);
            time(&currentTime);  /* get current time; same as: timer = time(NULL)  */

            double seconds = difftime(currentTime,prevTime);
            prevTime = currentTime;
            


            random.move();
            greenFile << kalmanGreen.update(seconds);
            blueFile << kalmanBlue.update(seconds);
            redFile << kalmanRed.update(seconds);



            if (shoot){
                Node n = kalmanGreen.getPos();
                double a = changeInAngle(purple,n);
                cout << a << endl;
                purple->angvel(0, a*3);
            }
            n = kalmanGreen.getPos();
        }
        purple->angvel(0,0);
        stopped = false;

        if (shoot) {
            double shootTime = 5;

            greenFile << kalmanGreen.predict((shootTime+8)*(sleepAmount));
            Node futurePos = kalmanGreen.getPos();
            time(&currentTime);  /* get current time; same as: timer = time(NULL)  */

            while (abs(changeInAngle(purple,futurePos)) > .005){
                usleep(10000);//sleepAmount);
                double a = changeInAngle(purple,futurePos);
                cout << a << endl;
                purple->angvel(0, a*3);           
            }
            purple->angvel(0,0);
            time(&prevTime);
            double seconds = difftime(currentTime,prevTime);
            
            double timeleft = shootTime - seconds - getBulletTime(purple, futurePos); 
            if (timeleft > 0)
                sleep(timeleft);
            
            cout << "timeleft: " << timeleft << endl;
            purple->shoot(0);

            kalmanGreen.reset();
        }
        else {
            break;
        }        
    }

    for (int i = 0; i < 10; i++){
        usleep(sleepAmount * 1000000);
        greenFile << kalmanGreen.predict(i*(sleepAmount));  
        blueFile << kalmanBlue.predict(i*(sleepAmount));
        redFile << kalmanRed.predict(i*(sleepAmount));      
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