//version with complimentary filter function, global angular_disp variable, global time0 time1 and dt variables, and implementation of
//complimentary filter in main method
//work done over summer 2017
//needs to be implemented into basic functions code at some point
//commented by BR - 1/26/2017


// trying to receive imu n pozyx data using new serial receive function; created while wrtiting PIDtest.c



#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <math.h>
#include <time.h>


#define Y_final 1.3
#define X_final .9
#define pi 22/7

#define t 0.1

//Declare Global Variables
float Pozyx[4];                             //Pozyx[3]: timestamp for pozyx
float PV[3];
float theta_bias=0;
float ax_bias=0, ay_bias=0, gyro_bias=0;
float IMU[3];                               //IMU[0]: x acceleration, IMU[1]: y acceleration, IMU[2]: angular velocity
float IMU_control[2];
float Epoch=0;
float angular_disp=0;
float time0, time1, dt = 0;

FILE *fp;


#define LEFT1 2  //thruster 2
#define LEFT2 6  //thruster 6
//#define LEFT3 4  //thruster 4
//#define LEFT4 27 //thruster 8

#define RIGHT1 0  //thruster 1
#define RIGHT2 5  //thruster 5
//#define RIGHT3 3  //thruster 3
//#define RIGHT4 26 //thruster 7


#define BACK1 3 //thruster 3
#define BACK2 27 //thruster 8

#define FRONT1 4 //thruster 4
#define FRONT2 26 //thruster 7

void MatrixMultiply(int m, int p, int n, float A[m][p], float B[p][n],  float C[m][n]);
void MatrixAdd(int m, int n, float A[m][n], float B[m][n], float C[m][n]);
void MatrixSubtract(int m, int n, float A[m][n], float B[m][n], float C[m][n]);
void MatrixTranspose(int m, int n, float A[m][n], float B[m][n]);
int MatrixInversion(int n, float *A, float *AInverse);
float covariance (float* x ,float* y, int size);
float variance (float* x, int size);
void navigate_left();
void navigate_right();
void navigate_fwd();
void navigate_back();
void navigate(float x, float y, float gyro);
float complimentary_filter(float IMU[3], float angular_disp, float dt);
void navigate_right_all();
void navigate_left_all();


double timer(void)
{
	long ns;
	double t_ns;
	double time;

	struct timespec spec;
	clock_gettime(CLOCK_REALTIME, &spec);

	ns = round(spec.tv_nsec);
	t_ns = ns;
	time = spec.tv_sec + (t_ns/1e9) - Epoch;

	//unsigned long time_in_micros = 100000 * s + spec.tv_usec;

	return time;
}

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 10;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}

void IMU_initialise()
{
	int i;
	char temp;
	char fd;
	char fs3[]= "@mode,A*2E\n";
	char fs2[]= "@asc_out,RPYIMU*26\n";
	char fs1[]= "@divider,5*38\n";
	char fs4[]= "@mode,C*2C\n";
	int c1 = sizeof(fs1);
	int c2 = sizeof(fs2);
	int c3 = sizeof(fs3);
	int c4 = sizeof(fs4);

	if ((fd = serialOpen ("/dev/ttyACM1", 115200)) < 0)
	{
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
	}

	for (i=0; i<100; i++)
	{
		serialGetchar (fd);
		fflush (stdout) ;
	}

	printf("\n");
    for (i=0; i<c2; i++)
	{
		serialPutchar(fd,fs2[i]);
	}

	for (i=0; i<c1; i++)
	{
		serialPutchar(fd,fs1[i]);
	}

	for (i=0; i<c3; i++)
	{
		serialPutchar(fd,fs3[i]);
	}

	for (i=0; i<c4; i++)
	{
		serialPutchar(fd,fs4[i]);
	}

	for (i=1; i<2000; i++)
	{
		serialGetchar (fd) ;
		fflush (stdout) ;
    }

	while(1)
	{
		temp = serialGetchar(fd);
		if(temp == '\n')
		{
			printf("\n\nIMU Initialization Done\n\n");
			break;
		}
	}
serialClose(fd);
}

     // ***************** IMU_GetData function *******************

void IMU_GetData(float* IMU)
{
    //Declare variables
	char fd;                                //buffer reader
	int	i=0,j=0,p=0,n5=0,n6=0,n8=0;         //indexing variables
	int N[15];
	char c[1000],ax[8],ay[8],Gz[9], temp;
	float X_acce,Y_acce,Gyro_z;             //variables where IMU data will be stored

	if ((fd = serialOpen ("/dev/ttyACM1", 115200)) < 0)
	{
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
	}

	//this loop uses the temp variable to read in values from the buffer
	//every time temp encounters a new line the loop breaks so that a subsequent
	//loop can read in the new buffer data
	while(1)
	{
		temp = serialGetchar(fd);
		if(temp == '\n')
		{
			break;
		}
	}

	//Read in all IMU data values from the buffer. Note size of c[i] is 1000
	while(1)
	{
		c[i] = serialGetchar(fd);
		if(c[i]=='\n')
			break;
		i++;
	}

	//want to see how many characters are in the array c[i].
	//c[i] is specified to be no larger than 1000.
//    printf("The size of c[i] is: %d", i);
	p = i;

    //This loop checks to see if any of the characters in the
    //IMU dataset array are ','. If they are this loop will use
    //array N[15] to mark the index number of the location of ','
	for(i=0; i<p; i++)
	{
		if(c[i]==',')
		{
			N[j]=i;
			j++;
		}
	}

    //This loop populates the ax array with acceleration values in
    //...you guessed it, the x direction
	for(i=(N[4]+1); i<N[5]; i++)
	{
		ax[n5]=c[i];
		n5++;
	}

    //This loop populates the ay array with acceleration values in
    //the y direction
	for(i=(N[5]+1); i<N[6]; i++)
	{
		ay[n6]=c[i];
		n6++;
	}

    //This loop populates the Gz array with angular velocity values
	for(i=(N[9]+1); i<N[10]; i++)
	{
		Gz[n8]=c[i];
		n8++;
	}

	//convert string values to decimal values using atof
	X_acce = atof(ax);
	Y_acce = atof(ay);
	Gyro_z = atof(Gz);

	//update IMU data array values
	IMU[0] = X_acce - ax_bias;          //x acceleration
	IMU[1] = Y_acce - ay_bias;          //y acceleration
	IMU[2] = Gyro_z - gyro_bias;        //angular velocity

	for(i=0;i<p;i++)
	{
		if(c[i]==0);
	}

	//reset array index values for next read
	n5=0;n6=0;n8=0;

	//close the serial port
	serialClose(fd);
}



void Pozyx_initialisation()
{
	char fc;
	int k; char temp;


	//note pozyx is designated to port 0. To ensure the the hardware recognizes
	//pozyx is actually in port 0 you need to connect it to the odroid BEFORE
	//connecting the IMU
	if ((fc = serialOpen ("/dev/ttyACM0", 115200)) < 0)
	{
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
	}

	//Read in data from the serial buffer. Then clear the buffer 600 times
	for (k=0;k<600;k++)
	{
		temp = serialGetchar (fc);
		putchar(temp);
		fflush (stdout) ;
    }
}

void Pozyx_Getdata()
{
	char *portname = "/dev/ttyACM0";
    int fd, i,k=0,j=0, pozyx_len;
    char buf[1000];
    char pozyx_buf[100]={0};
    int readlen, start;
    char x[5], y[5], theta[5];
    int nx=0,ny=0,nt=0;
    long int X=0, Y=0, angle =0;
    float pozyx_timestamp=0;
    float angleo=0;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("Error opening %s: %s\n", portname, strerror(errno));
	}

    //baudrate 115200, 8 bits, no parity, 1 stop bit
    set_interface_attribs(fd, B115200);
    //set_mincount(fd, 0);                /* set to pure timed read */


    readlen = read(fd, buf, sizeof(buf) - 1);
 //   printf("Readlen is: %d" , readlen);
	if (readlen > 0)
	{
		buf[readlen] = 0;
        pozyx_timestamp = timer();

		for(i=0;i<readlen;i++)
		{
//			printf("BUF[%d] is: %d",i,buf[i]);
		    if(buf[i] == '*')
		    start = i;
		    break;
		}

			for(i= start+1; i<readlen; i++)
            {
				pozyx_buf[j] = buf[i];
				j++;
				if(buf[i] == '$')
				{
					pozyx_len = j;
					break;
				}
			}

            for(i=0;i<pozyx_len;i++)
		    {
				if(pozyx_buf[i] == ',')
				{
					k=i;
					break;
				}
				else
				{
					x[nx] = pozyx_buf[i];
					//putchar(x[nx]);
					nx++;
				}
		    }

		   X=atoi(x);

		   for(i=k+1;i<pozyx_len;i++)
		   {
				if(pozyx_buf[i] == ',')
				{
					k=i;
					break;
				}
				else
				{
					y[ny] = pozyx_buf[i];
					//putchar(y[ny]);
					ny++;
				}
		    }

		   Y=atoi(y);

//		   printf("\n");printf("\n");
		   for(i=k+1;i<pozyx_len;i++)
		   {
				if(pozyx_buf[i] == '$')
				{
					k=i;
					break;
				}
				else
				{
					theta[nt] = pozyx_buf[i];
					//putchar(theta[nt]);
					nt++;
				}
		    }

			angle=atoi(theta);

			if(angle<0 || angle>360)
			angle = angleo;

			Pozyx[0] = X/10;
			Pozyx[1] = Y/10;
			Pozyx[2] = angle - theta_bias;
			Pozyx[3] = pozyx_timestamp;

			angleo=angle;

			for (i=0;i<5;i++)
			{
			   x[i] = 0;
			   y[i] = 0;
			   theta[i] = 0;
			}

			i=0;j=0;k=0;nx=0;ny=0;nt=0;

		   serialClose(fd);
	}

    else if (readlen < 0)
    {
		printf("Error from read: %d: %s\n", readlen, strerror(errno));
    }
        /* repeat read to get full message */
}


                  // ***************** KALMAN FILTER *******************

float A[4][4] = {{1,0,t,0},{ 0,1,0,t},{ 0,0,1,0},{ 0,0,0,1}};
float AT[4][4] = {{1,0,0,0},{0,1,0,0},{t,0,1,0},{0,t,0,1}};
float B[4][2]  = {{(t*t/2),0},{ 0,(t*t/2)},{ t,0},{ 0,t}};
float AX[4][1] = {{0},{0},{0},{0}};
float BIMU[4][1] = {{0},{0},{0},{0}};
float x_hat[4][1] = {{0},{0},{0},{0}};
float IMU_input[2][1] = {{0},{0}};
float AP[4][4];
float P0[4][4] = {{0,0,0,0},{ 0,0,0,0},{ 0,0,0,0},{ 0,0,0,0}};
float APA[4][4];
float Q[4][4] = {{0.00001,0.00001,0.00001,0.00001},{0.00001,0.00001,0.00001,0.00001},{0.00001,0.00001,0.00001,0.00001},{0.00001,0.00001,0.00001,0.00001}};
float P_hat[4][4];
float y_hat[2][1];
float z[2][1];
float H[2][4] = {{1,0,0,0},{ 0,1,0,0}};
float HT[4][2] = {{1,0},{0,1},{0,0},{0,0}};
float R[2][2] = {{0.01,0},{ 0,0.01}};
float HP[2][4];
float HPH[2][2];
float S_inv[2][2];
float K[4][2];
float KT[2][4];
float PH[4][2];
float Ky[4][1];
float I[4][4] = {{1,0,0,0},{ 0,1,0,0},{ 0,0,1,0},{ 0,0,0,1}};
float KH[4][4];
float iKH[4][4];
float P_n[4][4];
float Hx[2][1];
float X_n[4][1];
float S[2][2];
float Zxy[2][1] = {{0},{0}};
float VarX,VarY,VarXdot,VarYdot,Cov_XXdot,Cov_YYdot;
float theta;
float X0[4][1];
float F[4][4];
float FT[4][4];
float FP[4][4];
float FPF[4][4];
float KR[4][2];
float KRK[4][4];
int count =0;
float X_all[10][1];

void Kalman_position(float X_n[4][1], float IMU[3], float Pozyx[4])
{
	int i,j;

	Pozyx_Getdata(Pozyx);

    IMU_GetData(IMU);

    Zxy[0][0] = (Pozyx[0]/100) ;
    Zxy[1][0] = (Pozyx[1]/100) ;

    IMU_input[0][0] = IMU[0];
	IMU_input[1][0] = IMU[1];


	MatrixMultiply(4,4,1, A, X0,AX);
	MatrixMultiply(4,2,1, B, IMU_input,BIMU);
	MatrixAdd(4,1, AX, BIMU,x_hat);

	MatrixMultiply( 4, 4, 4, A, P0,AP);
	MatrixMultiply(4, 4, 4,AP, AT, APA);
	MatrixAdd(4,4, APA,Q,P_hat);

	MatrixMultiply(2,4, 1, H, x_hat,Hx);
	MatrixSubtract( 2,1,  Zxy, Hx,y_hat);

	MatrixMultiply(2, 4, 4,H, P_hat, HP);
	MatrixMultiply( 2, 4, 2, HP, HT,HPH);

	MatrixAdd(2,2, HPH,R,S);

	MatrixInversion(2,(float *)S, (float *)S_inv);

	MatrixMultiply(4, 4, 2, P_hat, HT, PH);
	MatrixMultiply(4,2,2, PH, S_inv,K);

	MatrixMultiply(4, 2, 1, K, y_hat, Ky);
	MatrixAdd(4,1,x_hat,Ky,X_n);

	MatrixMultiply(4,2,4 , K, H, KH);
	MatrixSubtract(4,4, I, KH, F);
	MatrixMultiply(4,4,4, F,P_hat, FP);
	MatrixTranspose(4,4,F,FT);
	MatrixTranspose(4,2, K, KT);
	MatrixMultiply(4,4,4,FP,FT , FPF);
	MatrixMultiply(4,2,2, K, R, KR);
	MatrixMultiply(4,2,4, KR, KT, KRK);
	MatrixAdd(4,4,FPF,KRK,P_n);


	for(i=0;i<4;i++)
	{
		X0[i][0] = X_n[i][0];
		for(j=0;j<4;j++)
		P0[i][j] = P_n[i][j];
	}

	count++;
//	printf("\ncount = %d\n",count);
}



float phi=0;

int main()
{
	//Debug
//	printf("here 1\n");



    long ns;
	double t_ns;
	float sumA=0, sumax=0,sumay=0, sumgy=0;
	int count=0, i;
	float x,y, gyro;


	wiringPiSetup () ;


	pinMode(LEFT1, OUTPUT);
	pinMode(LEFT2, OUTPUT);
	pinMode(RIGHT1,  OUTPUT);
	pinMode(RIGHT2, OUTPUT);
	pinMode(BACK1,  OUTPUT);
	pinMode(BACK2, OUTPUT);
	pinMode(FRONT1, OUTPUT);
	pinMode(FRONT2, OUTPUT);
//	pinMode(LEFT3, OUTPUT);
//	pinMode(LEFT4, OUTPUT);
//	pinMode(RIGHT3, OUTPUT);
//	pinMode(RIGHT4, OUTPUT);

	digitalWrite (LEFT1, HIGH) ;// off
	digitalWrite (LEFT2, HIGH) ;// off
	digitalWrite (RIGHT1, HIGH) ;// off
	digitalWrite (RIGHT2, HIGH) ;// off
	digitalWrite (BACK1, HIGH) ;// off
	digitalWrite (BACK2, HIGH) ;// off
	digitalWrite (FRONT1, HIGH);// off
	digitalWrite (FRONT2, HIGH);// off
//	digitalWrite (LEFT3, HIGH);
//	digitalWrite (LEFT4, HIGH);
//	digitalWrite (RIGHT3, HIGH);
//	digitalWrite (RIGHT4, HIGH);


	struct timespec spec;
	clock_gettime(CLOCK_REALTIME, &spec);

	ns = round(spec.tv_nsec);
	t_ns = ns;

	Epoch = spec.tv_sec + (t_ns/1e9);

	//Debug
//	printf("Calling POZYX Initialize");
	Pozyx_initialisation();
	//Debug
//	printf("Calling IMU Initialize");
	IMU_initialise();

//	printf("here 2\n");

	for(count=0;count<100;count++)
	{
//		printf("Loop");
		//printf("\nPozx Data\n");
		Pozyx_Getdata(Pozyx);
		//printf("\nIMU Data\n");
		IMU_GetData(IMU);

		sumA  = sumA  + Pozyx[2];
		sumax = sumax + IMU[0];
		sumay = sumay + IMU[1];
		sumgy = sumgy + IMU[2];

//		printf("\nX = %f, Y = %f, theta = %f, time = %f, sumA = %f\n", Pozyx[0], Pozyx[1], Pozyx[2], Pozyx[3], sumA);
//		printf("\nax = %f, ay = %f, gy = %f\n", IMU[0], IMU[1], IMU[2]);

		delay(100);
    }

 //   printf("\nax = %f, ay = %d\n", sumA, count);
    theta_bias = sumA/count;
    ax_bias = sumax/count;
	ay_bias = sumay/count;
	gyro_bias = sumgy/count;


//	printf("\n\ntheta_bias = %f, ax_bias = %f, ay_bias = %f, gyro_bias = %f\n\n",theta_bias, ax_bias, ay_bias,gyro_bias);

    Pozyx_Getdata(Pozyx);

	IMU_GetData(IMU);


    //this loop checks to see if the destination is reached -- if destination is not reached
    //signal is sent to relays to fire thrusters.
float tt = 0;
int first_loop = 1;
int at_Dest = 0;
//	for(i=0;i<35;i++)
fp = fopen("naviagte_output.txt", "w");
fclose(fp);
    while(!at_Dest)
	{

	X0[0][0] = (Pozyx[0]/100) ;
    X0[1][0] = (Pozyx[1]/100) ;
    X0[2][0] = 0;
    X0[3][0] = 0;

		Kalman_position(X_n, IMU, Pozyx);

		time1 = Pozyx[3];
		fp = fopen("naviagte_output.txt", "a+");
		fprintf(fp, "%f\t", time1);		//write time to naviagte_output.txt
		fclose(fp);

        if(first_loop)
        {
            time0 = time1;
		first_loop = 0;
        }

//       printf("Time 0 is: %f\n", time0);
//       printf("Time 1 is: %f\n", time1);

		dt = 0;
        dt = time1 - time0;
        time0 = time1;
//        printf("dt is: %f\n", dt);

//	printf("angular Disp Before function: %f\n", angular_disp);

/*		phi = Pozyx[2];
		if(phi<0)
		phi = phi +360;
*/
		//call complimentary fcn to get filtered angular displacement value
		angular_disp = complimentary_filter(IMU, angular_disp, dt);
		tt = tt + dt;
		//--------------//
//		printf("%f\t%f\n",angular_disp, tt);

		phi = angular_disp*-1;

		x = X_n[0][0];
		y = X_n[1][0];
		gyro = IMU[2];


//		fprintf(f,"%f\t %f\n", angular_disp, tt);


//		printf("\nPx = %f, X = %f: Py = %f, Y = %f, theta = %f, gyro_acce = %f",Pozyx[0], x*100, Pozyx[1], y*100,Pozyx[2], IMU[2] );

		if(y < (Y_final-0.1) || y > (Y_final+0.1) || x < (X_final-0.1) || x > (X_final+0.1))
		{
//			printf("\n x = %f, y = %f\n", x,y);
			printf("naviagtion fcn\n");
		    navigate(x,y,gyro );
		}

		else
		{
			printf("\n Destination is reached\n");

			break;
		}
//		fclose(fp);
	}

	return 0;
}



void MatrixMultiply(int m, int p, int n, float A[m][p], float B[p][n],  float C[m][n])
{

int i, j, k;
for (i=0;i<m;i++)
   for(j=0;j<n;j++)
      {
      C[i][j]=0;
      for (k=0;k<p;k++)
         C[i][j]= C[i][j]+A[i][k]*B[k][j];
      }
}


void MatrixAdd(int m, int n, float A[m][n], float B[m][n], float C[m][n])
{
int i, j;
for (i=0;i<m;i++)
   for(j=0;j<n;j++)
      C[i][j] = A[i][j] + B[i][j];
}


void MatrixSubtract(int m, int n, float A[m][n], float B[m][n], float C[m][n])
{

int i, j;
for (i=0;i<m;i++)
   for(j=0;j<n;j++)
      C[i][j] = A[i][j] - B[i][j];
}

void MatrixTranspose(int m, int n, float A[m][n], float B[m][n])
{

int i, j;
for (i=0;i<m;i++)
   for(j=0;j<n;j++)
      B[j][i]=A[i][j];
}

int MatrixInversion(int n, float *A, float *AInverse)
{
#define MAX_MATRIX 16

int i, j, iPass, imx, icol, irow;
float det, temp, pivot, factor;

if (n==1)
   {
   AInverse[0] = 1/A[0];
   return 1;
   }
det = 1;
for (i = 0; i < n; i++)
   {
   for (j = 0; j < n; j++)
      {
      AInverse[n*i+j] = 0;
      }
   AInverse[n*i+i] = 1;
   }
// The current pivot row is iPass.
// For each pass, first find the maximum element in the pivot column.
for (iPass = 0; iPass < n; iPass++)
   {
   imx = iPass;
   for (irow = iPass; irow < n; irow++)
      {
      if (fabs(A[n*irow+iPass]) > fabs(A[n*imx+iPass])) imx = irow;
      }
   // Interchange the elements of row iPass and row imx in both A and AInverse.
   if (imx != iPass)
      {
      for (icol = 0; icol < n; icol++)
         {
         temp = AInverse[n*iPass+icol];
         AInverse[n*iPass+icol] = AInverse[n*imx+icol];
         AInverse[n*imx+icol] = temp;
         if (icol >= iPass)
            {
            temp = A[n*iPass+icol];
            A[n*iPass+icol] = A[n*imx+icol];
            A[n*imx+icol] = temp;
            }
         }
      }
   // The current pivot is now A[iPass][iPass].
   // The determinant is the product of the pivot elements.
   pivot = A[n*iPass+iPass];
   det = det * pivot;
   if (det == 0)
      {
      return 0;
      }
   for (icol = 0; icol < n; icol++)
      {
      // Normalize the pivot row by dividing by the pivot element.
      AInverse[n*iPass+icol] = AInverse[n*iPass+icol] / pivot;
      if (icol >= iPass) A[n*iPass+icol] = A[n*iPass+icol] / pivot;
      }
   for (irow = 0; irow < n; irow++)
      // Add a multiple of the pivot row to each row.  The multiple factor
      // is chosen so that the element of A on the pivot column is 0.
      {
      if (irow != iPass) factor = A[n*irow+iPass];
      for (icol = 0; icol < n; icol++)
         {
         if (irow != iPass)
            {
            AInverse[n*irow+icol] -= factor * AInverse[n*iPass+icol];
            A[n*irow+icol] -= factor * A[n*iPass+icol];
            }
         }
      }
   }
   return 1;
}
       // ***************** Coariance function *******************


float covariance (float* x ,float* y, int size)
{
    float sumx = 0;
    float sumy = 0;
    float pro1 = 0;
    float Sum = 0;
    float pro = 0;
    float dif = 0;
    float cov=0;

    int i;

    for (i = 0; i < size; ++i)
    {
      sumx += x[i];
      sumy += y[i];

      pro1 = x[i]*y[i];
      Sum = Sum + pro1;
   }

   pro = (sumx*sumy)/size;
   dif = Sum - pro;
  if (size<2)
  return 0;
  else
  {
   cov = dif/(size-1);
   return cov;
  }

}

       // ***************** Variance function *******************

float variance (float* x, int size)
{
    float sumx = 0;
    float var = 0;
    float sum1 = 0;
    float mean =0;
    int i;

    for (i = 0; i < size; ++i)
    {
      sumx += x[i];
     }

   mean = sumx/size;

    for (i = 0; i < size; ++i)
    {
      sum1 = sum1 + ((x[i] - mean)*(x[i] - mean));
     }
    if (size<2)
    return 0;
    else
   {
    var = sum1/(float)(size-1);
    return var;
   }
   }

void navigate_left()
	{

//		printf("\nnavigation started left \n");

			digitalWrite(RIGHT1,LOW);
			digitalWrite(RIGHT2,LOW);
		           delay(50);
			  digitalWrite(RIGHT1,HIGH);
		             digitalWrite(RIGHT2,HIGH);


	}

void navigate_right()
	{

//		printf("\nnavigation started right \n");

			digitalWrite(LEFT1,LOW);
			digitalWrite(LEFT2,LOW);
		        delay(50);
			digitalWrite(LEFT1,HIGH);
		        digitalWrite(LEFT2,HIGH);
//printf("navigation ended right\n");
	}

void navigate_fwd()
	{

//		printf("\nnavigation started FWD\n");
			digitalWrite( BACK1,LOW);
			digitalWrite( BACK2,LOW);
                        delay(50);
			digitalWrite( BACK1,HIGH);
		         digitalWrite( BACK2,HIGH);

	}



void navigate_back()
	{
		digitalWrite( FRONT1, LOW);
		digitalWrite( FRONT2, LOW);
		delay(50);
		digitalWrite( FRONT1, HIGH);
		digitalWrite( FRONT2, HIGH);
	}

                     // ***************** NAVIGATION function *******************

void navigate(float x, float y, float gyro)
	{

//		printf("\nnavigation started\n");


		float Y1,Y2,X1,X2,theta, thetad,angle_diff;
		Y2 = Y_final;
		Y1 = y;
		X2 = X_final;
		X1 = x;
		printf("\nin d loop: xi = %f, yi = %f,\n xf = %f, yf = %f\n", X1,Y1,X2,Y2);

		fp = fopen("naviagte_output.txt", "a+");
		fprintf(fp, "%f\t", Y2);				//destination Y position
		fprintf(fp, "%f\t", X2);				//destination X position
		fprintf(fp, "%f\t", y);					//crrent y position
		fprintf(fp, "%f\t", x);					//current x position
		fclose(fp);

		if(Y1 == Y2)
            theta = 0;

		else if(X1 == X2)
            theta = pi/2;

		else
		{

		theta = atan((Y2-Y1)/(X2-X1));

	    }

		//quadrant 1
		if(Y2>Y1 && X2>X1)
		{
			theta = theta;
		}

		//quadrant 2
		else if(Y2>Y1 && X2<X1)
		{
			theta = pi + theta;
		}

		//quadrant 3
		else if(Y2<Y1 && X2<X1)
		{
			theta = pi + theta;
		}

		//quadrant 4
		else if(Y2<Y1 && X2>X1)
		{
			theta = 2*pi + theta;
		}


//		else printf("\n\n\nERROR\n\n\n");

		thetad = theta * 180/(22/7);

		theta = thetad;
		printf("\ntheta = %f, phi = %f\n", theta, phi);
/*
		while(theta > 360)
		{
			theta = theta - 360;
		}

		while(theta < 0)
		{
			theta = theta + 360;
		}

		while(phi > 360)
		{
			phi = phi - 360;
		}

		while(phi < 0)
		{
			phi = phi + 360;
		}
*/
        angle_diff = (phi-theta);
        fp = fopen("naviagte_output.txt", "a+");
		fprintf(fp, "%f\t", angle_diff);
		fclose(fp);

		printf("Phi: %f, Theta: %f, Angle Diff: %f\n", phi, theta, angle_diff);

		if(angle_diff < -10 || angle_diff > 10)
		{
		    printf("Gyro is: %f\n", gyro);
//			printf("\n angle_diff = %f\n", angle_diff);

		if ((angle_diff > 0 && angle_diff < 180) || (angle_diff > -360 && angle_diff < -180))
		{
			if(gyro>-0.1)
//            {
//                if(gyro > 0.5)
//                {
//                    printf("\n TURN LEFT ALL\n");
//                    navigate_left_all();
//                }
                printf("\n TURN LEFT\n");
                fp = fopen("naviagte_output.txt", "a+");
				fprintf(fp, "%s\n", "Turn Left");
                fclose(fp);
                //printf("\n TURN LEFT\n");
                //printf("\n TURN LEFT\n");
                navigate_left();
//            }


			// turn right for 250 ms
		}
		else if ((angle_diff > -180 && angle_diff < 0) || (angle_diff > 180 && angle_diff < 360))
		{
			if(gyro<0.1)
//            {
//                if(gyro < -0.5)
//                {
//                    printf("\n TURN RIGHT ALL\n");
//                    navigate_right_all();
//                }
                printf("\n TURN RIGHT\n");
                fp = fopen("naviagte_output.txt", "a+");
                fprintf(fp, "%s\n", "Turn Right");
                fclose(fp);
                //printf("\n TURN RIGHT\n");
                //printf("\n TURN RIGHT\n");
                navigate_right();
//            }

			// turn left for 250 ms
		}
	}

	  	else

		{
			printf("\n MOVE FORWARD\n");
			fp = fopen("naviagte_output.txt", "a+");
			fprintf(fp, "%s\n", "Move Forward");
			fclose(fp);
			//printf("\n MOVE FORWARD\n");
			//printf("\n MOVE FORWARD\n");
			//theta = theta + 360;
			//phi = phi + 360;
			navigate_fwd();
			// move forward for 250 ms

		}

	}

//Begin Complimentary Filter
//This function takes the values in the IMU array, and updates the angular
//displacement of the ABP. It is a simpler version of a Kalman filter
//The basic idea is that accelerometer data is accurate over a long period of
//time and gyroscopic data is accurate over a short period of time. So by using
//a combination of both data points we can get a more accurate reading for angular
//displacement. Note the values that we multiply "angular_disp" and "ang_accel" by
//can be changed. As long as their sum adds up to 1.
//Experimentally set to take 99.9% of gyroscope data and only 0.1% of accelerometer
//note: clockwiserotation is positive.
float complimentary_filter(float IMU[3], float angular_disp, float dt)
{
    float ang_accel;
    //Check to see if ABP is rotating. If it's stationary, then we return
    //the previous angular displacement value.
    //note that this line is an adjustment to complimentary filter to only adjust
    //angle if IMU angular velocity is moving greater than
    //an experimentally determined band - i.e. if bot is not rotating,
    //do not take new IMU data, but hold previous value
	if(IMU[2]<0.8 && IMU[2] > -0.8)
	{
//		printf("Inside Gyro = 0\n");
//		printf("IMU[2] is: %f\n", IMU[2]);
//		printf("gyro is 0 and angular disp is: %f\n", angular_disp);
		return angular_disp;
	}
//below is traditional complimentary filter
    angular_disp = angular_disp + (IMU[2]*dt);
//	printf("Gyro is: %f\n", IMU[2]);
//	printf("IMU[2]*dt is: %f\n", IMU[2]*dt);
    ang_accel = atan2f((float) IMU[0], (float) IMU[1])*180/pi;
//	printf("ang_accel: %f\n", ang_accel);
    angular_disp = (0.9999*angular_disp) + (0.0001*ang_accel);
//	printf("Angular Displacement is: %f---------------------------------------\n", angular_disp);
	return angular_disp;

}

//void navigate_right_all()
//{
//    digitalWrite(LEFT1,LOW);
//    digitalWrite(LEFT2,LOW);
//    digitalWrite(LEFT3,LOW);
//    digitalWrite(LEFT4,LOW);
//    delay(200);
//    digitalWrite(LEFT1,HIGH);
 //   digitalWrite(LEFT2,HIGH);
//    digitalWrite(LEFT3,HIGH);
//    digitalWrite(LEFT4,HIGH);
//}

//void navigate_left_all()
//{
//    digitalWrite(RIGHT1,LOW);
//    digitalWrite(RIGHT2,LOW);
//    digitalWrite(RIGHT3,LOW);
//    digitalWrite(RIGHT4,LOW);
//    delay(200);
 //   digitalWrite(RIGHT1,HIGH);
 //   digitalWrite(RIGHT2,HIGH);
//    digitalWrite(RIGHT3,HIGH);
//    digitalWrite(RIGHT4,HIGH);
//}
