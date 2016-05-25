#define _USE_MATH_DEFINES // for C++

#include <cmath>
#include <iostream>
#include <cassert>
#include <fstream>
#include <vector>
#include <sys/stat.h>

using namespace std;

struct RawData
{
	float time; //s
	unsigned int id;
	unsigned int voltage; //mV

	float gx;
	float gy;
	float gz;

	float ax;
	float ay;
	float az;

	float mx;
	float my;
	float mz;

	RawData()
	{

	}
	RawData(char** data)
	{
		time = atof((char*)data[0]);
		id = atoi((char*)data[1]);
		voltage = atoi((char*)data[2]);
		gx = atof((char*)data[3]);
		gy = atof((char*)data[4]);
		gz = atof((char*)data[5]);
		ax = atof((char*)data[6]);
		ay = atof((char*)data[7]);
		az = atof((char*)data[8]);
		mx = atof((char*)data[9]);
		my = atof((char*)data[10]);
		mz = atof((char*)data[11]);
	}
};

struct EulerData
{
	float psi, theta, phi;

	EulerData(float* angles)
	{
		psi = angles[0];
		theta = angles[1];
		phi = angles[2];
	}
};

struct ProcessedData
{
	float time; //s
	unsigned int id;
	unsigned int voltage; //mV

	float gx;
	float gy;
	float gz;

	float ax;
	float ay;
	float az;

	float mx;
	float my;
	float mz;

	float psi;
	float theta;
	float phi;

	ProcessedData(RawData raw, EulerData angles)
	{
		time = raw.time;
		id = raw.id;
		voltage = raw.voltage;
		gx = raw.gx;
		gy = raw.gy;
		gz = raw.gz;
		ax = raw.ax;
		ay = raw.ay;
		az = raw.az;
		mx = raw.mx;
		my = raw.my;
		mz = raw.mz;
		psi = angles.psi;
		theta = angles.theta;
		phi = angles.phi;
	}
};

bool dataTest(char** data)
{
	if (data == NULL) return false;
	if (data[0] == NULL) return false;
	//if (sizeof(data) / sizeof(data[0]) < 12) return false;

	return true;
}

bool fileExists(const std::string& filename)
{
	struct stat buf;
	if (stat(filename.c_str(), &buf) != -1)
	{
		return true;
	}
	return false;
}

float invsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y = number;
	i = *(long *)&y;                       // evil floating point bit level hacking
	i = 0x5f3759df - (i >> 1);               // what the fuck? 
	y = *(float *)&i;
	y = y * (threehalfs - (x2 * y * y));   // 1st iteration
	//y = y * (threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

	return y;
}

float q[4] = { 1.0, 0.0, 0.0, 0.0 };
float exInt = 0.0;
float eyInt = 0.0;
float ezInt = 0.0;

#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.1f) // 2 * integral gain
float twoKp = twoKpDef;
float twoKi = twoKiDef;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

#define SCALE_GYRO 2000
#define SCALE_MAG_SENSITIVITY 0.00029f
#define SCALE_ACCEL 16

RawData computeQ(RawData raw, RawData prevRaw)
{
	//Scale raw values
	const float gRes = (SCALE_GYRO) / 32768.0;
	const float mRes = SCALE_MAG_SENSITIVITY;
	const float aRes = (SCALE_ACCEL) / 32768.0;

	raw.ax *= aRes;
	raw.ay *= aRes;
	raw.az *= aRes;
	raw.gx *= gRes;
	raw.gy *= gRes;
	raw.gz *= gRes;
	raw.mx *= mRes;
	raw.my *= mRes;
	raw.mz *= mRes;

	//Convert gyro values from deg/s to rad/s
	//raw.gx *= M_PI / 180;
	//raw.gy *= M_PI / 180;
	//raw.gz *= M_PI / 180;

	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
	float recipNorm;

	//get sample frequency
	double sampleFreq = 1.0 / ((raw.time - prevRaw.time)); //in seconds

	//Boring variables
	float q0q0 = q[0] * q[0];
	float q0q1 = q[0] * q[1];
	float q0q2 = q[0] * q[2];
	float q0q3 = q[0] * q[3];
	float q1q1 = q[1] * q[1];
	float q1q2 = q[1] * q[2];
	float q1q3 = q[1] * q[3];
	float q2q2 = q[2] * q[2];
	float q2q3 = q[2] * q[3];
	float q3q3 = q[3] * q[3];
	float qa, qb, qc;

	//Magnetometer compensation
	if((raw.mx != 0.0f) && (raw.my != 0.0f) && (raw.mz != 0.0f)) {
		float hx, hy, bx, bz;
		float halfwx, halfwy, halfwz;

		// Normalise magnetometer measurement
		recipNorm = invsqrt(raw.mx * raw.mx + raw.my * raw.my + raw.mz * raw.mz);
		raw.mx *= recipNorm;
		raw.my *= recipNorm;
		raw.mz *= recipNorm;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (raw.mx * (0.5f - q2q2 - q3q3) + raw.my * (q1q2 - q0q3) + raw.mz * (q1q3 + q0q2));
		hy = 2.0f * (raw.mx * (q1q2 + q0q3) + raw.my * (0.5f - q1q1 - q3q3) + raw.mz * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f * (raw.mx * (q1q3 - q0q2) + raw.my * (q2q3 + q0q1) + raw.mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of magnetic field
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (raw.my * halfwz - raw.mz * halfwy);
		halfey = (raw.mz * halfwx - raw.mx * halfwz);
		halfez = (raw.mx * halfwy - raw.my * halfwx);
	}

	//Accelorometer compensation
	if ((raw.ax != 0.0) && (raw.ay != 0.0) && (raw.az != 0.0))
	{
		double halfvx, halfvy, halfvz;

		// Normalise accelerometer measurement
		recipNorm = invsqrt(raw.ax * raw.ax + raw.ay * raw.ay + raw.az * raw.az);
		raw.ax *= recipNorm;
		raw.ay *= recipNorm;
		raw.az *= recipNorm;

		// Estimated direction of gravity
		halfvx = q[1]*q[3] - q[0]*q[2];
		halfvy = q[0]*q[1] + q[2]*q[3];
		halfvz = q[0]*q[0] - 0.5f + q[3]*q[3];

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += (raw.ay * halfvz - raw.az * halfvy);
		halfey += (raw.az * halfvx - raw.ax * halfvz);
		halfez += (raw.ax * halfvy - raw.ay * halfvx);
	}

	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
		// Compute and apply integral feedback if enabled
		if (twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			raw.gx += integralFBx;  // apply integral feedback
			raw.gy += integralFBy;
			raw.gz += integralFBz;
		}
		else {
			integralFBx = 0.0f; // prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		raw.gx += twoKp * halfex;
		raw.gy += twoKp * halfey;
		raw.gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	raw.gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
	raw.gy *= (0.5f * (1.0f / sampleFreq));
	raw.gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * raw.gx - qc * raw.gy - q[3] * raw.gz);
	q[1] += (qa * raw.gx + qc * raw.gz - q[3] * raw.gy);
	q[2] += (qa * raw.gy - qb * raw.gz + q[3] * raw.gx);
	q[3] += (qa * raw.gz + qb * raw.gy - qc * raw.gx);

	// Normalise quaternion
	recipNorm = invsqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;

	return raw;
}

EulerData computeEuler(float* q)
{
	float angles[3];
	angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1) * 180 / M_PI; // psi
	angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180 / M_PI; // theta
	angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * 180 / M_PI; // phi
	return EulerData(angles);
}

#define LINE_BUFFER 128

int main(int argc, char **argv)
{
	if (argc < 2) {
		cout << "./RawAnalyze [data.csv]" << endl;
		return 0;
	}

	ifstream file(argv[1]);
	if (!file.good())
	{
		cout << "File not found!" << endl;
		return 0;
	}

	if (fileExists("analyzed.csv"))
	{
		if (!remove("analyzed.csv"))
		{
			cout << "ERROR! Cannot replace analysis file!" << endl;
		}
	}

	ofstream output("analyzed.csv");
	output << "Time (s),Sample,Power (mV),AccelX (Gs),AccelY (Gs),AccelZ (Gs),GyroX (DPS),GyroY (DPS),GyroZ (DPS)," <<
		"MagX (Gs),MagY (Gs),MagZ (Gs),Psi (deg),Theta (deg),Phi (deg)" << endl;

	//Read every line in the file
	char line[LINE_BUFFER];
	char* item;
	char** data = new char*[12];
	RawData prevRaw = RawData();
	unsigned long lineid = 0;

	//Ignore the first line
	file.getline(line, LINE_BUFFER);

	//Read every line
	while (!file.eof())
	{
		//Split the line into columns
		file.getline(line, LINE_BUFFER);
		int col = 0;
		item = strtok(line, ",");
		data[0] = item;
		while (item != NULL && (col < 12))
		{
			item = strtok(NULL, ",");
			data[++col] = item;
		}

		//Get raw data instance from row
		if (!dataTest(data))
		{
			cout << "WARNING! Bad data on count " << lineid << "!" << endl;
			lineid++;
			continue;
		}
		RawData raw(data);

		//Scrap first sample
		if (lineid == 0)
		{
			lineid++;
			prevRaw = raw;
			continue;
		}

		//Get quaternion from raw data
		RawData computed = computeQ(raw, prevRaw);

		//Compute Euler angles
		EulerData angles = computeEuler(q);

		//Get processed data instance
		ProcessedData processed(computed, angles);

		//Output data to file
		output << processed.time << "," << processed.id << "," << processed.voltage << "," << processed.ax << "," << processed.ay << "," <<
			processed.az << "," << processed.gx << "," << processed.gy << "," << processed.gz <<
			"," << processed.mx << "," << processed.my << "," << processed.mz << "," << processed.psi << "," << processed.theta <<
			"," << processed.phi << endl;

		lineid++;
	}

	file.close();
	output.close();

	cout << "Done!" << endl;
}