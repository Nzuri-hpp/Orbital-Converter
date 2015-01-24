// OrbitalConverter.cpp : Defines the entry point for the console application.

// Copyright 2015 Злобин Данил

/*This program is free software : you can redistribute it and / or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <math.h>
#include <limits>
#include <time.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>

using namespace std;

#define __3D__               3u
#define __FLOAT_TYPE__       double
#define __EPSILON__          DBL_EPSILON///DBL_EPSILO if __FLOAT_TYPE__ determined as double
#define __GRAVITY_CONSTANT__ 6.67384e-11
#define __PI__               3.14159265358979323846
#define __PI_2__	     1.57079632679489661923
#define __OUTPUT_PRECISION__ 20u

bool convertOrbitalData(__FLOAT_TYPE__ orbitalSemimajorAxis,   __FLOAT_TYPE__ eccentricity, 
	                 __FLOAT_TYPE__ inclination,            __FLOAT_TYPE__ argumentOfPeriapsis, 
			__FLOAT_TYPE__ ascendingNodeLongitude, __FLOAT_TYPE__ eraMeanAnomaly,     
			unsigned currentYYYYMMDD,               unsigned eraYYYYMMDD,
			__FLOAT_TYPE__ gravityParameter,       __FLOAT_TYPE__ *outputStorage);

inline void greeting(void);
string help(string inputParameter);
inline string stringDetect(const string inputString, const string inputParameter);

int _tmain(int argc, _TCHAR* argv[])
{
#define __INVITATION__ endl << ">"

	string inputString;
	string inputParameter;
	string answer;
	
	greeting();

	while (true)
	{
		cout << __INVITATION__;
		cin >> inputString;
		if (inputString[0] == '/') goto LABEL_NOPARAM;
		cin >> inputParameter;

LABEL_NOPARAM:
		answer = stringDetect(inputString, inputParameter);
		
		if ( strcmp(answer.c_str(),"exit") == 0 ) goto LABEL_EXIT;
		cout << answer;
	}

LABEL_EXIT:
	system("pause");
	return 0;
}

void           copyVector         (const __FLOAT_TYPE__ *sourceVector, __FLOAT_TYPE__ *toVector);
void           getRotateMatrix    (__FLOAT_TYPE__ angleRad, const __FLOAT_TYPE__* normVector, __FLOAT_TYPE__ outRotateMatrix[][__3D__]);
void           applyRotate        (__FLOAT_TYPE__* myVector, const __FLOAT_TYPE__ rotateMatrix[][__3D__], __FLOAT_TYPE__* outVector);
void           rotateVector       (__FLOAT_TYPE__* myVector, __FLOAT_TYPE__* axisVector, __FLOAT_TYPE__ angleRad, bool normalization = true);
__FLOAT_TYPE__ diffYYYYMMDD       (unsigned date2, unsigned date1);
__FLOAT_TYPE__ getMeanAnomaly     (__FLOAT_TYPE__ gravityParameter, __FLOAT_TYPE__ orbitalSemimajorAxis, unsigned currentYYYYMMDD, unsigned eraYYYYMMDD, __FLOAT_TYPE__ eraMeanAnomaly);
__FLOAT_TYPE__ getEccentricAnomaly(__FLOAT_TYPE__ meanAnomaly, __FLOAT_TYPE__ eccentricity, bool *fail = nullptr);
__FLOAT_TYPE__ getTrueAnomaly     (__FLOAT_TYPE__ eccentricity, __FLOAT_TYPE__ eccentricAnomaly);
__FLOAT_TYPE__ getDistance        (__FLOAT_TYPE__ orbitalSemimajorAxis, __FLOAT_TYPE__ eccentricity, __FLOAT_TYPE__ eccentricAnomaly);
__FLOAT_TYPE__ getOrbitalSpeed    (__FLOAT_TYPE__ gravityParameter, __FLOAT_TYPE__ distance, __FLOAT_TYPE__ orbitalSemimajorAxis);
__FLOAT_TYPE__ getAngleDS         (__FLOAT_TYPE__ distance, __FLOAT_TYPE__ orbitalSemimajorAxis, __FLOAT_TYPE__ eccentricity);
void           vectorProduct      (__FLOAT_TYPE__ *resultVector, const __FLOAT_TYPE__ *vectorA, const __FLOAT_TYPE__ *vectorB);

bool convertOrbitalData (__FLOAT_TYPE__ orbitalSemimajorAxis,   __FLOAT_TYPE__ eccentricity,
						__FLOAT_TYPE__ inclination,            __FLOAT_TYPE__ argumentOfPeriapsis,
						__FLOAT_TYPE__ ascendingNodeLongitude, __FLOAT_TYPE__ eraMeanAnomaly,
							  unsigned currentYYYYMMDD,              unsigned eraYYYYMMDD,
					    __FLOAT_TYPE__ gravityParameter,       __FLOAT_TYPE__ *outputStorage)
{
	const __FLOAT_TYPE__ axis_X[__3D__] = { 1.0, 0.0, 0.0 };
	const __FLOAT_TYPE__ axis_Y[__3D__] = { 0.0, 1.0, 0.0 };
	const __FLOAT_TYPE__ axis_Z[__3D__] = { 0.0, 0.0, 1.0 };

#define __X 0u
#define __Y 1u
#define __Z 2u

#define __POS_X_Idx   0u
#define __POS_Y_Idx   1u
#define __POS_Z_Idx   2u
#define __SPEED_X_Idx 3u
#define __SPEED_Y_Idx 4u
#define __SPEED_Z_Idx 5u
///#define __ELLIPSE_NORMAL_X_Idx
///#define __DIRECT_TO_PERIAPSIS_X_Idx

#define __a    orbitalSemimajorAxis
#define __e    eccentricity
#define __i    inclination
#define __w	   argumentOfPeriapsis
#define __O    ascendingNodeLongitude
#define __MA_0 eraMeanAnomaly
	
	bool fail;

	__FLOAT_TYPE__ rotateAxis[__3D__];
	__FLOAT_TYPE__ direction[__3D__];
	__FLOAT_TYPE__ ellipseNormal[__3D__];
	__FLOAT_TYPE__ tempVector[__3D__];

	copyVector(axis_X, direction);
	copyVector(axis_Z, rotateAxis);

	rotateVector(direction, rotateAxis, __O, false);
	
	copyVector(direction, tempVector);
	copyVector(axis_Z, rotateAxis);
	rotateVector(tempVector, rotateAxis, __PI_2__, false);
	copyVector(direction, rotateAxis);
	rotateVector(tempVector, rotateAxis, __i, false);

	vectorProduct(ellipseNormal, direction, tempVector);

	rotateVector(direction, ellipseNormal, __w, false);

	__FLOAT_TYPE__ meanAnomaly;
	__FLOAT_TYPE__ eccentricAnomaly;
	__FLOAT_TYPE__ trueAnomaly;

	meanAnomaly = getMeanAnomaly(gravityParameter, __a, currentYYYYMMDD, eraYYYYMMDD, __MA_0);
	eccentricAnomaly = getEccentricAnomaly(meanAnomaly, __e, &fail);
	trueAnomaly = getTrueAnomaly(__e, eccentricAnomaly);

	rotateVector(direction, ellipseNormal, trueAnomaly, false);

	__FLOAT_TYPE__ distance;

	distance = getDistance(__a, __e, eccentricAnomaly);

	outputStorage[__POS_X_Idx] = distance * direction[__X];
	outputStorage[__POS_Y_Idx] = distance * direction[__Y];
	outputStorage[__POS_Z_Idx] = distance * direction[__Z];

	__FLOAT_TYPE__ orbitalSpeedDirect[__3D__];
	__FLOAT_TYPE__ orbitalSpeed;
	__FLOAT_TYPE__ speedAngle;

	orbitalSpeed = getOrbitalSpeed(gravityParameter, distance, __a);
	speedAngle = getAngleDS(distance, __a, __e);
	///cout << orbitalSpeed << " " << __a << " " << __e << " " << distance << endl;///DEGUB INFO
	copyVector(direction, orbitalSpeedDirect);
	rotateVector(orbitalSpeedDirect, ellipseNormal, speedAngle, false);

	outputStorage[__SPEED_X_Idx] = orbitalSpeed * orbitalSpeedDirect[__X];
	outputStorage[__SPEED_Y_Idx] = orbitalSpeed * orbitalSpeedDirect[__Y];
	outputStorage[__SPEED_Z_Idx] = orbitalSpeed * orbitalSpeedDirect[__Z];


#undef __X 
#undef __Y 
#undef __Z 

#undef __POS_X_Idx   
#undef __POS_Y_Idx   
#undef __POS_Z_Idx   
#undef __SPEED_X_Idx 
#undef __SPEED_Y_Idx 
#undef __SPEED_Z_Idx  


#undef __a   
#undef __e  
#undef __i   
#undef __w	  
#undef __O   
#undef __MA_0 

	return fail;
}

void copyVector(const __FLOAT_TYPE__ *sourceVector, __FLOAT_TYPE__ *toVector)
{
	for (ptrdiff_t i = 0; i < __3D__; i++)
	{
		toVector[i] = sourceVector[i];
	}
}
							   
void getRotateMatrix(__FLOAT_TYPE__ angleRad, const __FLOAT_TYPE__* normVector, __FLOAT_TYPE__ outRotateMatrix[][__3D__])
{
#define __X normVector[0u]
#define __Y normVector[1u]
#define __Z normVector[2u]

#define __Mtx outRotateMatrix
#define __o angleRad

	__FLOAT_TYPE__ cos_o = cos(__o);
	__FLOAT_TYPE__ sin_o = sin(__o);

	__Mtx[0][0] = cos_o + (1 - cos_o)*__X*__X;       /*|*/  __Mtx[0][1] = (1 - cos_o)*__X*__Y - sin_o*__Z; /*|*/ __Mtx[0][2] = (1 - cos_o)*__X*__Z + sin_o*__Y;
	__Mtx[1][0] = (1 - cos_o)*__X*__Y + sin_o*__Z;   /*|*/  __Mtx[1][1] = cos_o + (1 - cos_o)*__Y*__Y;     /*|*/ __Mtx[1][2] = (1 - cos_o)*__Y*__Z - sin_o*__X;
	__Mtx[2][0] = (1 - cos_o)*__X*__Z - sin_o*__Y;   /*|*/	__Mtx[2][1] = (1 - cos_o)*__Z*__Y + sin_o*__X; /*|*/ __Mtx[2][2] = cos_o + (1 - cos_o)*__Z*__Z;

#undef __X
#undef __Y
#undef __Z

#undef __Mtx
#undef __o
}

void applyRotate(__FLOAT_TYPE__* myVector, const __FLOAT_TYPE__ rotateMatrix[][__3D__], __FLOAT_TYPE__* outVector)
{
	for (ptrdiff_t ind = 0; ind < __3D__; ind++)
	{
		outVector[ind] = 0;
	}

	for (ptrdiff_t i = 0; i < __3D__; i++)
	{
		for (ptrdiff_t j = 0; j < __3D__; j++)
		{
			outVector[i] += rotateMatrix[i][j] * myVector[j];
		}
	}
}

void rotateVector(__FLOAT_TYPE__* myVector,  __FLOAT_TYPE__* axisVector, __FLOAT_TYPE__ angleRad, bool normalization)
{
	__FLOAT_TYPE__ temp[__3D__];
	__FLOAT_TYPE__ rotateMatrix[__3D__][__3D__];

#define __X axisVector[0u]
#define __Y axisVector[1u]
#define __Z axisVector[2u]

	const __FLOAT_TYPE__ length = sqrt(__X*__X + __Y*__Y + __Z*__Z);

	if (length > 1.0 + __EPSILON__ && normalization)
	{
		for (ptrdiff_t i = 0; i < __3D__; i++)
		{
			if (axisVector[i] > __EPSILON__ && axisVector[i] < 0.0 - __EPSILON__ && length > __EPSILON__)
			{
				axisVector[i] /= length;
			}
		}
	}

	getRotateMatrix(angleRad, axisVector, rotateMatrix);
	applyRotate(myVector, rotateMatrix, temp);

	for (ptrdiff_t i = 0; i < __3D__; i++)
	{
		myVector[i] = temp[i];
	}

#undef __X
#undef __Y
#undef __Z
}

__FLOAT_TYPE__ diffYYYYMMDD(unsigned date2, unsigned date1)
{
#define __TIME_START_YEAR__ 1900u
#define __FIRST_JANUARY_SHIFT__ 1u

	struct tm date1tm = { 0, 0, 0, date1 % 100u, (date1 % 10000u) / 100u - __FIRST_JANUARY_SHIFT__, date1 / 10000u - __TIME_START_YEAR__ };
	struct tm date2tm = { 0, 0, 0, date2 % 100u, (date2 % 10000u) / 100u - __FIRST_JANUARY_SHIFT__, date2 / 10000u - __TIME_START_YEAR__ };

	__FLOAT_TYPE__ differenceSec = (__FLOAT_TYPE__)difftime(mktime(&date2tm), mktime(&date1tm));

#undef __TIME_START_YEAR__
#undef __FIRST_JANUARY_SHIFT__

	return differenceSec;
}

__FLOAT_TYPE__ getMeanAnomaly(__FLOAT_TYPE__ gravityParameter, __FLOAT_TYPE__ orbitalSemimajorAxis, unsigned currentYYYYMMDD, unsigned eraYYYYMMDD, __FLOAT_TYPE__ eraMeanAnomaly)
{
#define __a_pow3 pow(orbitalSemimajorAxis, 3.0)

	__FLOAT_TYPE__ meanMotion = sqrt(gravityParameter/ __a_pow3);
	__FLOAT_TYPE__ diffTime = diffYYYYMMDD(currentYYYYMMDD, eraYYYYMMDD);
	__FLOAT_TYPE__ meanAnomaly = eraMeanAnomaly + meanMotion * diffTime;

#undef __a_pow3

	return meanAnomaly;
}

__FLOAT_TYPE__ getEccentricAnomaly(__FLOAT_TYPE__ meanAnomaly, __FLOAT_TYPE__ eccentricity, bool *fail)
{
#define __ACCURACY 1.0e-5 //+ __EPSILON__
#define __MAX_STEPS 1000000u

	__FLOAT_TYPE__ currentEccentricAnomaly = meanAnomaly;

#define __e eccentricity
#define __E currentEccentricAnomaly
#define __M meanAnomaly

	bool accStop = false;
	///unsigned debugStepN = 0;
	__FLOAT_TYPE__ temp = __M - __e * sin(__M);

	for (ptrdiff_t step = 0; step < __MAX_STEPS && !accStop; step++)
	{
		__E = __M + __e * sin(__E);
		temp = __E - __e * sin(__E);

		if (abs(__M - temp) < __ACCURACY) accStop = true;

		///debugStepN++;
	}

	if (fail != nullptr) *fail = !accStop;

#undef __ACCURACY
#undef __MAX_STEPS

#undef __e
#undef __E
#undef __M

	return currentEccentricAnomaly;
}

__FLOAT_TYPE__ getTrueAnomaly(__FLOAT_TYPE__ eccentricity, __FLOAT_TYPE__ eccentricAnomaly)
{
#define __e eccentricity
#define __E eccentricAnomaly

	__FLOAT_TYPE__ tan_trueAnomaly_2 = sqrt((1.0 + __e) / (1.0 - __e)) * tan(__E / 2.0);
	__FLOAT_TYPE__ trueAnomaly = 2.0 * atan(tan_trueAnomaly_2);

#undef __e
#undef __E

	return trueAnomaly;
}

__FLOAT_TYPE__ getDistance(__FLOAT_TYPE__ orbitalSemimajorAxis, __FLOAT_TYPE__ eccentricity, __FLOAT_TYPE__ eccentricAnomaly)
{
#define __a orbitalSemimajorAxis
#define __e eccentricity
#define __E eccentricAnomaly

	__FLOAT_TYPE__ distance = __a * (1.0 - __e * cos(__E));

#undef __a
#undef __e
#undef __E

	return distance;
}

__FLOAT_TYPE__ getOrbitalSpeed(__FLOAT_TYPE__ gravityParameter, __FLOAT_TYPE__ distance, __FLOAT_TYPE__ orbitalSemimajorAxis)
{
#define __a orbitalSemimajorAxis
#define __r distance
#define __u gravityParameter

	__FLOAT_TYPE__ orbitalSpeed = sqrt(__u * (2.0 / __r - 1.0 / __a) );

#undef __a
#undef __r
#undef __u

	return orbitalSpeed;
}

__FLOAT_TYPE__ getAngleDS(__FLOAT_TYPE__ distance, __FLOAT_TYPE__ orbitalSemimajorAxis, __FLOAT_TYPE__ eccentricity)
{
#define __a orbitalSemimajorAxis
#define __a_pow2 pow(__a, 2.0)
#define __r distance
#define __e eccentricity
#define __e_pow2 pow(__e, 2.0)

	__FLOAT_TYPE__ sin_fi = sqrt(__a_pow2 * (1.0 - __e_pow2) / (__r * (2.0 * __a - __r) ) );
	__FLOAT_TYPE__ angleDS = __PI__ - asin(sin_fi);

#undef __a
#undef __a_pow2
#undef __r
#undef __e
#undef __e_pow2

	return angleDS;
}

void vectorProduct(__FLOAT_TYPE__ *resultVector, const __FLOAT_TYPE__ *vectorA, const __FLOAT_TYPE__ *vectorB)
{
#define __X 0u
#define __Y 1u
#define __Z 2u

#define __A_X vectorA[__X]
#define __A_Y vectorA[__Y]
#define __A_Z vectorA[__Z]

#define __B_X vectorB[__X]
#define __B_Y vectorB[__Y]
#define __B_Z vectorB[__Z]

	resultVector[__X] = __A_Y*__B_Z - __A_Z*__B_Y;
	resultVector[__Y] = __A_Z*__B_X - __A_X*__B_Z;
	resultVector[__Z] = __A_X*__B_Y - __A_Y*__B_X;

#undef __X 
#undef __Y 
#undef __Z 

#undef __A_X 
#undef __A_Y 
#undef __A_Z 

#undef __B_X 
#undef __B_Y 
#undef __B_Z 
}

// ----------INTERFACE--------------
//

inline void greeting(void)
{
	setlocale(LC_ALL, "RUS");
	cout << "--------------------------Orbital Converter версия 1.0--------------------------" << endl;
	cout << ">Добро пожаловать! Вас приветствует конвертер орбитальных данных." << endl;
	cout << ">Исходный текст программы доступен на https://github.com/Nzuri-hpp/Orbital-Converter" << endl;
	cout << ">Для конвертации из файла введите batch путь_к_файлу. " << endl;
	cout << ">Для получения справки по формату ввода введите /help. " << endl;
}

string help(string inputParameter)
{
#define __IP inputParameter.c_str()

	ofstream fileHelp("example.txt");
	
	if (fileHelp.is_open())
	{
	          fileHelp << 
			
"/ --------------------------------------------Orbit Converter 1.0-----------------------------------------------------------\n\
/ -------------------------[ФАЙЛ СОЗДАН АВТОМАТИЧЕСКИ ПО ЗАПРОСУ ПОЛЬЗОВАТЕЛЯ]----------------\n\
/ /\n\
/ ДАННЫЙ ФАЙЛ ПРЕДНАЗНАЧЕН ДЛЯ ДЕМОНСТРАЦИИ ВОЗМОЖНОСТЕЙ ПРОГРАММЫ\n\
/ АВТОР НЕ ГАРАНТИРУЕТ ДОСТОВЕРНОСТЬ ПРЕДОСТАВЛЕННОЙ ИНФОРМАЦИИ ОБ\n\
/ ОРБИТАХ ТЕЛ СОЛНЕЧНОЙ СИСТЕМЫ И НЕ НЕСЕТ ОТВЕТСТВЕННОСТИ ЗА ВОЗМОЖНЫЕ УБЫТКИ,\n\
/ СВЯЗАННЫЕ С ИСПОЛЬЗОВАНИЕМ ДАННОЙ ДЕМОНСТРАЦИИ И ПО ВЦЕЛОМ.\n\
/ / --------------------------------------------------------------------------------------------------------------------------\n\
/ Входной файл элементов орбит тел Солнечной системы - ДЕМОНСТРАЦИОННЫЙ ПРИМЕР\n\
/ /\n\
/ a - большая полуось[метров]\n\
/ e - эксцентриситет[безразмерен]\n\
/ i - наклонение[радиан]\n\
/ w - аргумент перицентра[радиан]\n\
/ O - долгота восходящего узла[радиан]\n\
/ M0 - средняя аномалия в начальную эпоху[радиан]\n\
/ Date - дата конечного положения тела после конвертации[формат YYYYMMDD]\n\
/ elementsDate - дата начальной эпохи[формат YYYYMMDD]\n\
/ m - масса конвертируемого тела[кг]\n\
/ M - масса парного тела в задаче двух тел(прим. - звезды)[кг]\n\
/ /\n\
/ (!) *десятичный разделитель - точка '.'\n\
/ ------------------------------------------------------------------------\n\
/ Описание должно быть создано согласно шаблону :\n\
/\n\
/ PlanetName a e i w O M0 Date elementsDate m M\n\
/\n\
/ (Пожалуйста, между параметрами используйте только один пробел, иначе будет вызвана ошибка формата данных.\n\
/ После символа начала строки - комментария(длинна не более 1024 символов) ставьте минимум один пробел\n\
/ Так же обратите внимание, что пустые строки недопустимы(даже в конце файла).\n\
/ Или используйте символ комментария, или определяйте входные данные)\n\
/ ---------------------------------------- - [СЕКЦИЯ ВХОДНЫХ ДАННЫХ]----------------------------------------------------------\n\
Mercury 57909227000 0.20563593 0.1221730476 0.50831456 0.8435467745 6.089848205 20010101 20010101 3.33022e+23 1.98892e+30\n\
/ /\n\
Venus 108208930000 0.00680000 0.0592465977 0.95735306 1.3381559800 1.109723972 20010101 20010101 4.8685e+24 1.98892e+30\n\
/ /\n\
Earth 149598261000 0.01671123 3.4906585e-6 1.99330266 6.086650063 1.929566208 20010101 20010101 5.9726e+24 1.98892e+30\n\
/ /\n\
Mars 2.2794382e+11 0.0933941 0.0322992376 4.99971031 0.865308761 3.355030713 20010101 20010101 0.64185e+24 1.98892e+30\n\
/ /\n\
Jupiter 7.785472e+11 0.048775 0.0179768913 4.80080736 1.755035901 1.143921953 20010101 20010101 1.8986e+27 1.98892e+30\n\
/ /\n\
Saturn 143344937e+4 0.05572322 0.0433888852 5.86454822 1.983441223 1.094963132 20010101 20010101 5.6846e+26 1.98892e+30\n\
/ /\n\
Uranus 2876679082e+3 0.04440559 0.0134836459 1.68496386 1.291365989 5.548063098 20010101 20010101 8.6832e+25 1.98892e+30\n\
/ /\n\
Neptune 4503443661e+3 0.01121427 0.0308569848 4.63641223 2.300244645 5.365358541 20010101 20010101 1.0243e+26 1.98892e+30\n\
/ /\n\
Pluto 5.906376272e+12 0.24880766 0.2991799771 1.98554398 1.925158728 4.196314319 20010101 20010101 1.305e+22 1.98892e+30";
				
			fileHelp.close();
			return ">В дирректории программы создан демонстрационный файл example.txt";
	}
	else
	{
		return ">Ошибка создания файла example.txt";
	}
	
}

string batchConvert(string fileName)
{
	char tempLine[1024];
	string tempName;
	__FLOAT_TYPE__ tempFloat;
	unsigned       tempUnsigned;

	ifstream inputFile(fileName.c_str());
	stringstream streamConvertDD;
	string convertDoneData;

	vector<string>         bodyName;
	vector<__FLOAT_TYPE__> SemimajorAxis;
	vector<__FLOAT_TYPE__> eccentricity;
	vector<__FLOAT_TYPE__> inclination;
	vector<__FLOAT_TYPE__> argumentOfPeriapsis;
	vector<__FLOAT_TYPE__> ascendingNodeLongitude;
	vector<__FLOAT_TYPE__> eraMeanAnomaly;
	vector<unsigned>       currentYYYYMMDD;
	vector<unsigned>       eraYYYYMMDD;
	vector<__FLOAT_TYPE__> mass;
	vector<__FLOAT_TYPE__> starMass;

	if (!inputFile) return ">Ошибка чтения из файла. Проверьте верность имени файла.\n";
	
	while (!inputFile.eof())
	{
		inputFile >> tempName;

		if (tempName[0u] == '/')
		{
			inputFile.getline(tempLine, 1024);
		}
		else
		{
			bodyName.push_back(tempName);

			if (inputFile.eof()) goto LABEL_EOF;
			inputFile >> tempFloat;
			SemimajorAxis.push_back(tempFloat);

			if (inputFile.eof()) goto LABEL_EOF;
			inputFile >> tempFloat;
			eccentricity.push_back(tempFloat);

			if (inputFile.eof()) goto LABEL_EOF;
			inputFile >> tempFloat;
			inclination.push_back(tempFloat);

			if (inputFile.eof()) goto LABEL_EOF;
			inputFile >> tempFloat;
			argumentOfPeriapsis.push_back(tempFloat);

			if (inputFile.eof()) goto LABEL_EOF;
			inputFile >> tempFloat;
			ascendingNodeLongitude.push_back(tempFloat);

			if (inputFile.eof()) goto LABEL_EOF;
			inputFile >> tempFloat;
			eraMeanAnomaly.push_back(tempFloat);

			if (inputFile.eof()) goto LABEL_EOF;
			inputFile >> tempUnsigned;
			currentYYYYMMDD.push_back(tempUnsigned);

			if (inputFile.eof()) goto LABEL_EOF;
			inputFile >> tempUnsigned;
			eraYYYYMMDD.push_back(tempUnsigned);

			if (inputFile.eof()) goto LABEL_EOF;
			inputFile >> tempFloat;
			mass.push_back(tempFloat);

			if (inputFile.eof()) goto LABEL_EOF;
			inputFile >> tempFloat;
			starMass.push_back(tempFloat);
		}
	}

#define __a SemimajorAxis
#define __e eccentricity
#define __i inclination
#define __w argumentOfPeriapsis
#define __O ascendingNodeLongitude
#define __MA_0 eraMeanAnomaly

	if ((bodyName.size() + __a.size() + __e.size() + __i.size() + __w.size() + __O.size() + __MA_0.size() + currentYYYYMMDD.size() + eraYYYYMMDD.size() + mass.size() + starMass.size()) != 11u * __a.size())
	{
	LABEL_EOF:
		return ">Неверный формат входных данных!\n";
	}

	ptrdiff_t convertSize = __a.size();

	vector<__FLOAT_TYPE__> position_X;
	vector<__FLOAT_TYPE__> position_Y;
	vector<__FLOAT_TYPE__> position_Z;

	vector<__FLOAT_TYPE__> speed_X;
	vector<__FLOAT_TYPE__> speed_Y;
	vector<__FLOAT_TYPE__> speed_Z;

	__FLOAT_TYPE__ tempStorage[2u * __3D__];

#define __POS_X_Idx   0u
#define __POS_Y_Idx   1u
#define __POS_Z_Idx   2u
#define __SPEED_X_Idx 3u
#define __SPEED_Y_Idx 4u
#define __SPEED_Z_Idx 5u

#define __G __GRAVITY_CONSTANT__

	bool fail = false;
	bool tempFail;

	for (ptrdiff_t i = 0; i < convertSize; i++)
	{
		tempFail = convertOrbitalData(__a[i], __e[i], __i[i], __w[i], __O[i], __MA_0[i], currentYYYYMMDD[i], eraYYYYMMDD[i], __G * (mass[i] + starMass[i]), tempStorage);

		if (tempFail) fail = true;

		position_X.push_back(tempStorage[__POS_X_Idx]);
		position_Y.push_back(tempStorage[__POS_Y_Idx]);
		position_Z.push_back(tempStorage[__POS_Z_Idx]);

		speed_X.push_back(tempStorage[__SPEED_X_Idx]);
		speed_Y.push_back(tempStorage[__SPEED_Y_Idx]);
		speed_Z.push_back(tempStorage[__SPEED_Z_Idx]);
	}

	if ( (position_X.size() + position_Y.size() + position_Z.size() + speed_X.size() + speed_Y.size() + speed_Z.size() ) != 6u * convertSize)
	{
		return ">Неизвестная ошибка в процессе конвертации! Приносим извинения.\n";
	}

	for (ptrdiff_t i = 0; i < convertSize; i++)
	{
		streamConvertDD << setprecision(__OUTPUT_PRECISION__) << bodyName[i] << " " << mass[i] << " " << position_X[i] << " " << position_Y[i] << " " << position_Z[i] << " "
						<< speed_X[i] << " " << speed_Y[i] << " " << speed_Z[i] << endl;
	}
	
	string outputName;
	ptrdiff_t length = fileName.size();
	ptrdiff_t dotPosition;
	
	for (ptrdiff_t i = 0; i < length; i++)
	{
		if (fileName[length - i - 1u] == '.')
			dotPosition = length - i - 1;
	}
	outputName = fileName;
	outputName.resize(dotPosition);
	outputName += "_convDone.dat";

	ofstream outputFile(outputName);
	if (!outputFile)
	{
		return "Ошибка создания файла вывода.\n";
	}

	convertDoneData = streamConvertDD.str();
	outputFile << "// -------------------------------------------Orbital Converter 1.0---------------------------------------------\n";
	outputFile << "// Файл сгенерирован автоматически программай Orbital Converter 1.0, на основе пользовательских данных.\n";
	outputFile << "// Данные расположены согласно шаблону: bodyName mass position_x position_y position_z speed_x speed_y speed_z\n";
	outputFile << "// -------------------------------------------------------------------------------------------------------------\n";
	outputFile << convertDoneData;
	outputFile.close();

	return "Конвертация успешно завершена! Файл " + outputName + " создан!\n";
}

inline string stringDetect(const string inputString, const string inputParameter)
{
#define __IS inputString.c_str()

	string answer;

	if ( strcmp(__IS, "/help") == 0 )
	{
		answer = help(inputParameter);
	}
	else if ( strcmp(__IS, "/exit") == 0 )
	{
		answer = "exit";
	}
	else if ( strcmp(__IS, "batch") == 0 )
	{
		answer = batchConvert(inputParameter);
	}
	
	return answer;
}
