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

	float yaw; //in degrees
	float pitch;
	float roll;

	ProcessedData(RawData raw, float _yaw, float _pitch, float _roll)
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
		yaw = _yaw;
		pitch = _pitch;
		roll = _roll;
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

#define SCALE_GYRO 2000
#define SCALE_MAG_SENSITIVITY 0.00029f
#define SCALE_ACCEL 16
//#define DECLINATION 4 + 2/60 // Declination (degrees) in Austin, Texas

ProcessedData computeYPR(RawData partial)
{
	RawData raw(partial);

	//Perform transformation
	//(az,ay,-ax,-mz,-mx,my)
	float tmp[6] = { partial.ax, partial.ay, partial.az, partial.mx, partial.my, partial.mz };
	partial.ax = tmp[2];
	partial.ay = tmp[1];
	partial.az = -tmp[0];
	partial.mx = -tmp[5];
	partial.my = -tmp[3];
	partial.mz = -tmp[4];
	
	//Compute roll and pitch
	float roll = atan2(partial.ay, partial.az);
	float pitch = atan2(-partial.ax, sqrt(partial.ay * partial.ay + partial.az * partial.az));

	float yaw; //aka "heading"
	if (partial.my == 0)
		yaw = (partial.mx < 0) ? 180.0 : 0;
	else
		yaw = atan2(partial.mx, partial.my);

	//yaw -= (DECLINATION) * M_PI / 180;

	if (yaw > M_PI) yaw -= (M_PI_2);
	else if (yaw < -M_PI) yaw += (M_PI_2);
	else if (yaw < 0) yaw += M_PI_2;

	//Convert everything from radians to degrees:
	yaw *= 180.0 / M_PI; //heading
	pitch *= 180.0 / M_PI;
	roll *= 180.0 / M_PI;

	return ProcessedData(raw, yaw, pitch, roll);;
}

ProcessedData process(RawData raw, RawData prevRaw)
{
	//Compute YPR
	ProcessedData processed = computeYPR(raw);

	//Scale raw values
	const float gRes = (SCALE_GYRO) / 32768.0;
	const float mRes = SCALE_MAG_SENSITIVITY;
	const float aRes = (SCALE_ACCEL) / 32768.0;

	processed.ax *= aRes;
	processed.ay *= aRes;
	processed.az *= aRes;
	processed.gx *= gRes;
	processed.gy *= gRes;
	processed.gz *= gRes;
	processed.mx *= mRes;
	processed.my *= mRes;
	processed.mz *= mRes;

	return processed;
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
			//return 1;
		}
	}

	ofstream output("analyzed.csv");
	output << "Time (s),Sample,Power (mV),AccelX (Gs),AccelY (Gs),AccelZ (Gs),GyroX (DPS),GyroY (DPS),GyroZ (DPS)," <<
		"MagX (Gs),MagY (Gs),MagZ (Gs),Yaw (deg),Pitch (deg),Roll (deg)" << endl;

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

		ProcessedData processed = process(raw, prevRaw);

		//Output data to file
		output << processed.time << "," << processed.id << "," << processed.voltage << "," << processed.ax << "," << processed.ay << "," <<
			processed.az << "," << processed.gx << "," << processed.gy << "," << processed.gz <<
			"," << processed.mx << "," << processed.my << "," << processed.mz << "," << processed.yaw << "," << processed.pitch <<
			"," << processed.roll << endl;

		lineid++;
	}

	file.close();
	output.close();

	cout << "Done!" << endl;
	return 0;
}