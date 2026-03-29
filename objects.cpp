/*********************************************************************
	Simulation obiektów fizycznych ruchomych np. samochody, statki, roboty, itd.
	+ obs³uga obiektów statycznych np. env.
	**********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <windows.h>
#include <gl\gl.h>
#include <gl\glu.h>
#include "objects.h"
#include "graphics.h"

extern FILE* f;
extern Environment env;

extern bool if_ID_visible;

// Licznik instancji - zeby kazdy klient dostawal inne ID nawet przy tym samym seedzie
static int instance_counter = 0;

MovableObject::MovableObject()             // konstruktor                   
{
	// ID bazowane na czasie + licznik instancji - unikamy duplikatow i ogromnych liczb
	srand((unsigned)time(NULL) + instance_counter * 137);
	iID = 0;  // ID w zakresie 100-999, zawsze dodatnie
	instance_counter++;
	fprintf(f, "my_car->iID = %d\n", iID);

	// zmienne zwi¹zame z akcjami kierowcy
	F = Fb = 0;	// si³y dzia³aj¹ce na obiekt 
	breaking_factor = 0;			// stopieñ hamowania
	steer_wheel_speed = 0;  // prêdkoœæ krêcenia kierownic¹ w rad/s
	if_keep_steer_wheel = 0;  // informacja czy kieronica jest trzymana

	// sta³e samochodu
	mass_own = 12.0;			// masa obiektu [kg]
	length = 9.0;
	width = 5.0;
	height = 1.7;
	clearance = 0.0;     // wysokoœæ na której znajduje siê podstawa obiektu
	front_axis_dist = 1.0;     // odleg³oœæ od przedniej osi do przedniego zderzaka 
	back_axis_dist = 0.2;       // odleg³oœæ od tylniej osi do tylniego zderzaka
	steer_wheel_ret_speed = 0.5; // prêdkoœæ powrotu kierownicy w rad/s

	is_collided = false;
	collision_point = Vector3(0, 0, 0);
	bounding_radius = 4.5;

	// parametry stateu auta - spawn blisko centrum do testow kolizji
	state.steering_angle = 0;
	state.vPos.y = clearance + height / 2 + 20;
	state.vPos.x = (float)(rand() % 16) - 8.0f;   // losowo -8..+8
	state.vPos.z = (float)(rand() % 16) - 8.0f;   // losowo -8..+8
	quaternion qObr = AsixToQuat(Vector3(0, 1, 0), 0.1 * PI / 180.0);
	state.qOrient = qObr * state.qOrient;
}

MovableObject::~MovableObject()            // destruktor
{
}

void MovableObject::ChangeState(ObjectState __state)
{
	state = __state;
}

ObjectState MovableObject::State()
{
	return state;
}



void MovableObject::Simulation(float dt)
{
	if (dt == 0) return;

	if (is_collided) {
		Vector3 to_obstacle = (collision_point - state.vPos).znorm();
		if ((state.vV ^ to_obstacle) > 0) {
			state.vV = Vector3(0, 0, 0);
			state.vA = Vector3(0, 0, 0);
		}
	}

	float friction = 3.0;
	float friction_rot = friction;
	float friction_roll = 0.15;
	float elasticity = 0.5;
	float g = 9.81;
	float Fy = mass_own * 9.81;

	Vector3 dir_forward = state.qOrient.rotate_vector(Vector3(1, 0, 0));
	Vector3 dir_up = state.qOrient.rotate_vector(Vector3(0, 1, 0));
	Vector3 dir_right = state.qOrient.rotate_vector(Vector3(0, 0, 1));

	Vector3 vV_forward = dir_forward * (state.vV ^ dir_forward),
		vV_right = dir_right * (state.vV ^ dir_right),
		vV_up = dir_up * (state.vV ^ dir_up);

	Vector3 vV_ang_forward = dir_forward * (state.vV_ang ^ dir_forward),
		vV_ang_right = dir_right * (state.vV_ang ^ dir_right),
		vV_ang_up = dir_up * (state.vV_ang ^ dir_up);

	if (steer_wheel_speed != 0)
		state.steering_angle += steer_wheel_speed * dt;
	else
		if (state.steering_angle > 0)
		{
			if (!if_keep_steer_wheel)
				state.steering_angle -= steer_wheel_ret_speed * dt;
			if (state.steering_angle < 0) state.steering_angle = 0;
		}
		else if (state.steering_angle < 0)
		{
			if (!if_keep_steer_wheel)
				state.steering_angle += steer_wheel_ret_speed * dt;
			if (state.steering_angle > 0) state.steering_angle = 0;
		}
	if (state.steering_angle > PI * 60.0 / 180) state.steering_angle = PI * 60.0 / 180;
	if (state.steering_angle < -PI * 60.0 / 180) state.steering_angle = -PI * 60.0 / 180;

	if (Fy > 0)
	{
		float V_ang_turn = 0;
		if (state.steering_angle != 0)
		{
			float Rs = sqrt(length * length / 4 + (fabs(length / tan(state.steering_angle)) + width / 2) * (fabs(length / tan(state.steering_angle)) + width / 2));
			V_ang_turn = vV_forward.length() * (1.0 / Rs);
		}
		Vector3 vV_ang_turn = dir_up * V_ang_turn * (state.steering_angle > 0 ? 1 : -1);
		Vector3 vV_ang_up2 = vV_ang_up + vV_ang_turn;
		if (vV_ang_up2.length() <= vV_ang_up.length())
		{
			if (vV_ang_up2.length() > V_ang_turn)
				vV_ang_up = vV_ang_up2;
			else
				vV_ang_up = vV_ang_turn;
		}
		else
		{
			if (vV_ang_up.length() < V_ang_turn)
				vV_ang_up = vV_ang_turn;
		}

		float V_ang_friction = Fy * friction_rot * dt / mass_own / 1.0;
		float V_ang_up = vV_ang_up.length() - V_ang_friction;
		if (V_ang_up < V_ang_turn) V_ang_up = V_ang_turn;
		vV_ang_up = vV_ang_up.znorm() * V_ang_up;
	}


	Fy = mass_own * g * dir_up.y;
	if (Fy < 0) Fy = 0;
	float Fh = Fy * friction * breaking_factor;

	float V_up = vV_forward.length();
	if (V_up < 0) V_up = 0;

	float V_right = vV_right.length();
	if (V_right < 0) V_right = 0;

	Vector3 P = state.vPos + dir_forward * (length / 2 - front_axis_dist) - dir_right * width / 2 - dir_up * height / 2,
		Q = state.vPos + dir_forward * (length / 2 - front_axis_dist) + dir_right * width / 2 - dir_up * height / 2,
		R = state.vPos + dir_forward * (-length / 2 + back_axis_dist) - dir_right * width / 2 - dir_up * height / 2,
		S = state.vPos + dir_forward * (-length / 2 + back_axis_dist) + dir_right * width / 2 - dir_up * height / 2;

	Vector3 Pt = P, Qt = Q, Rt = R, St = S;
	Pt.y = env.DistFromGround(P.x, P.z); Qt.y = env.DistFromGround(Q.x, Q.z);
	Rt.y = env.DistFromGround(R.x, R.z); St.y = env.DistFromGround(S.x, S.z);
	Vector3 normPQR = normal_vector(Pt, Rt, Qt), normPRS = normal_vector(Pt, Rt, St), normPQS = normal_vector(Pt, St, Qt),
		normQRS = normal_vector(Qt, Rt, St);

	float sryPQR = ((Qt ^ normPQR) - normPQR.x * state.vPos.x - normPQR.z * state.vPos.z) / normPQR.y,
		sryPRS = ((Pt ^ normPRS) - normPRS.x * state.vPos.x - normPRS.z * state.vPos.z) / normPRS.y,
		sryPQS = ((Pt ^ normPQS) - normPQS.x * state.vPos.x - normPQS.z * state.vPos.z) / normPQS.y,
		sryQRS = ((Qt ^ normQRS) - normQRS.x * state.vPos.x - normQRS.z * state.vPos.z) / normQRS.y;
	float sry = sryPQR; Vector3 norm = normPQR;
	if (sry > sryPRS) { sry = sryPRS; norm = normPRS; }
	if (sry > sryPQS) { sry = sryPQS; norm = normPQS; }
	if (sry > sryQRS) { sry = sryQRS; norm = normQRS; }

	Vector3 vV_ang_horizontal = Vector3(0, 0, 0);
	if ((P.y <= Pt.y + height / 2 + clearance) || (Q.y <= Qt.y + height / 2 + clearance) ||
		(R.y <= Rt.y + height / 2 + clearance) || (S.y <= St.y + height / 2 + clearance))
	{
		Vector3 v_rotation = -norm.znorm() * dir_up * 0.6;
		vV_ang_horizontal = v_rotation / dt;
	}

	Vector3 vAg = Vector3(0, -1, 0) * g;

	if ((P.y <= Pt.y + height / 2 + clearance) + (Q.y <= Qt.y + height / 2 + clearance) +
		(R.y <= Rt.y + height / 2 + clearance) + (S.y <= St.y + height / 2 + clearance) > 2)
		vAg = vAg + dir_up * (dir_up ^ vAg) * -1;
	else
		Fy = 0;

	state.vV_ang = vV_ang_up + vV_ang_horizontal;

	float h = sry + height / 2 + clearance - state.vPos.y;
	float V_podbicia = 0;
	if ((h > 0) && (state.vV.y <= 0.01))
		V_podbicia = 0.5 * sqrt(2 * g * h);
	if (h > 0) state.vPos.y = sry + height / 2 + clearance;

	Vector3 dvPos = state.vV * dt + state.vA * dt * dt / 2;
	state.vPos = state.vPos + dvPos;

	if (state.vPos.x < -env.field_size * env.number_of_columns / 2) state.vPos.x += env.field_size * env.number_of_columns;
	else if (state.vPos.x > env.field_size * (env.number_of_columns - env.number_of_columns / 2)) state.vPos.x -= env.field_size * env.number_of_columns;
	if (state.vPos.z < -env.field_size * env.number_of_rows / 2) state.vPos.z += env.field_size * env.number_of_rows;
	else if (state.vPos.z > env.field_size * (env.number_of_rows - env.number_of_rows / 2)) state.vPos.z -= env.field_size * env.number_of_rows;

	Vector3 vV_pop = state.vV;

	state.vV = vV_forward.znorm() * V_up + vV_right.znorm() * V_right + vV_up +
		Vector3(0, 1, 0) * V_podbicia + state.vA * dt;

	if ((P.y <= Pt.y + height / 2 + clearance) || (Q.y <= Qt.y + height / 2 + clearance) ||
		(R.y <= Rt.y + height / 2 + clearance) || (S.y <= St.y + height / 2 + clearance))
	{
		Vector3 dvV = vV_up + dir_up * (state.vA ^ dir_up) * dt;
		if ((dir_up.znorm() - dvV.znorm()).length() > 1)
			state.vV = state.vV - dvV;
	}

	state.vA = (dir_forward * F + dir_right * Fb) / mass_own * (Fy > 0)
		- vV_forward.znorm() * (Fh / mass_own + friction_roll * Fy / mass_own) * (V_up > 0.01)
		- vV_right.znorm() * friction * Fy / mass_own * (V_right > 0.01)
		+ vAg;

	Vector3 w_obrot = state.vV_ang * dt + state.vA_ang * dt * dt / 2;
	quaternion q_obrot = AsixToQuat(w_obrot.znorm(), w_obrot.length());
	state.qOrient = q_obrot * state.qOrient;
}

void MovableObject::DrawObject()
{
	glPushMatrix();

	glTranslatef(state.vPos.x, state.vPos.y + clearance, state.vPos.z);

	Vector3 k = state.qOrient.AsixAngle();
	Vector3 k_znorm = k.znorm();

	glRotatef(k.length() * 180.0 / PI, k_znorm.x, k_znorm.y, k_znorm.z);
	glTranslatef(-length / 2, -height / 2, -width / 2);
	glScalef(length, height, width);

	glCallList(Auto);
	GLfloat Surface[] = { 2.0f, 2.0f, 1.0f, 1.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, Surface);
	if (if_ID_visible) {
		glRasterPos2f(0.30, 1.20);
		glPrint("%d", iID);
	}
	glPopMatrix();
}




//**********************
//   Obiekty nieruchome
//**********************
Environment::Environment()
{
	field_size = 35;

	int wynik = ReadMap("map.txt");
	if (wynik == 0)
		wynik = ReadMap("..//map.txt");
	if (wynik == 0)
		fprintf(f, "Cannot open map.txt file. Check if this file exists in project directory!\n");


	d = new float** [number_of_rows];
	for (long i = 0; i < number_of_rows; i++) {
		d[i] = new float* [number_of_columns];
		for (long j = 0; j < number_of_columns; j++) d[i][j] = new float[4];
	}
	Norm = new Vector3 * *[number_of_rows];
	for (long i = 0; i < number_of_rows; i++) {
		Norm[i] = new Vector3 * [number_of_columns];
		for (long j = 0; j < number_of_columns; j++) Norm[i][j] = new Vector3[4];
	}

	fprintf(f, "height_map env: number_of_rows = %d, number_of_columns = %d\n", number_of_rows, number_of_columns);
}

Environment::~Environment()
{
	for (long i = 0; i < number_of_rows * 2 + 1; i++) delete height_map[i];
	delete height_map;
	for (long i = 0; i < number_of_rows; i++) {
		for (long j = 0; j < number_of_columns; j++) delete d[i][j];
		delete d[i];
	}
	delete d;
	for (long i = 0; i < number_of_rows; i++) {
		for (long j = 0; j < number_of_columns; j++) delete Norm[i][j];
		delete Norm[i];
	}
	delete Norm;
}

float Environment::DistFromGround(float x, float z)
{
	float x_begin = -field_size * number_of_columns / 2,
		z_begin = -field_size * number_of_rows / 2;

	long k = (long)((x - x_begin) / field_size),
		w = (long)((z - z_begin) / field_size);

	if (k < 0) while (k < 0) k += number_of_columns;
	else if (k > number_of_columns - 1) while (k > number_of_columns - 1) k -= number_of_columns;
	if (w < 0) while (w < 0) w += number_of_rows;
	else if (w > number_of_rows - 1) while (w > number_of_rows - 1) w -= number_of_rows;

	Vector3 B = Vector3(x_begin + (k + 0.5) * field_size, height_map[w * 2 + 1][k], z_begin + (w + 0.5) * field_size);
	enum tr { ABC = 0, ADB = 1, BDE = 2, CBE = 3 };
	int triangle = 0;
	if ((B.x > x) && (fabs(B.z - z) < fabs(B.x - x))) triangle = ADB;
	else if ((B.x < x) && (fabs(B.z - z) < fabs(B.x - x))) triangle = CBE;
	else if ((B.z > z) && (fabs(B.z - z) > fabs(B.x - x))) triangle = ABC;
	else triangle = BDE;

	float dd = d[w][k][triangle];
	Vector3 N = Norm[w][k][triangle];
	float y;
	if (N.y > 0) y = (-dd - N.x * x - N.z * z) / N.y;
	else y = 0;

	return y;
}

void Environment::DrawInitialisation()
{
	enum tr { ABC = 0, ADB = 1, BDE = 2, CBE = 3 };
	float x_begin = -field_size * number_of_columns / 2,
		z_begin = -field_size * number_of_rows / 2;
	Vector3 A, B, C, D, E, N;
	glNewList(EnvironmentMap, GL_COMPILE);
	glBegin(GL_TRIANGLES);

	for (long w = 0; w < number_of_rows; w++)
		for (long k = 0; k < number_of_columns; k++)
		{
			A = Vector3(x_begin + k * field_size, height_map[w * 2][k], z_begin + w * field_size);
			B = Vector3(x_begin + (k + 0.5) * field_size, height_map[w * 2 + 1][k], z_begin + (w + 0.5) * field_size);
			C = Vector3(x_begin + (k + 1) * field_size, height_map[w * 2][k + 1], z_begin + w * field_size);
			D = Vector3(x_begin + k * field_size, height_map[(w + 1) * 2][k], z_begin + (w + 1) * field_size);
			E = Vector3(x_begin + (k + 1) * field_size, height_map[(w + 1) * 2][k + 1], z_begin + (w + 1) * field_size);

			Vector3 AB = B - A;
			Vector3 BC = C - B;
			N = (AB * BC).znorm();
			glNormal3f(N.x, N.y, N.z);
			glVertex3f(A.x, A.y, A.z);
			glVertex3f(B.x, B.y, B.z);
			glVertex3f(C.x, C.y, C.z);
			d[w][k][ABC] = -(B ^ N);
			Norm[w][k][ABC] = N;
			Vector3 AD = D - A;
			N = (AD * AB).znorm();
			glNormal3f(N.x, N.y, N.z);
			glVertex3f(A.x, A.y, A.z);
			glVertex3f(D.x, D.y, D.z);
			glVertex3f(B.x, B.y, B.z);
			d[w][k][ADB] = -(B ^ N);
			Norm[w][k][ADB] = N;
			Vector3 BD = D - B;
			Vector3 DE = E - D;
			N = (BD * DE).znorm();
			glNormal3f(N.x, N.y, N.z);
			glVertex3f(B.x, B.y, B.z);
			glVertex3f(D.x, D.y, D.z);
			glVertex3f(E.x, E.y, E.z);
			d[w][k][BDE] = -(B ^ N);
			Norm[w][k][BDE] = N;
			Vector3 CB = B - C;
			Vector3 BE = E - B;
			N = (CB * BE).znorm();
			glNormal3f(N.x, N.y, N.z);
			glVertex3f(C.x, C.y, C.z);
			glVertex3f(B.x, B.y, B.z);
			glVertex3f(E.x, E.y, E.z);
			d[w][k][CBE] = -(B ^ N);
			Norm[w][k][CBE] = N;
		}

	glEnd();
	glEndList();
}

int Environment::ReadMap(char filename[128])
{
	int mode_reading_things = 0, mode_reading_map = 0, mode_reading_row = 0,
		nr_of_row_point = -1, nr_of_column_point = -1;
	height_map = NULL;

	this->number_of_rows = this->number_of_columns = 0;

	FILE* pl = fopen(filename, "r");

	if (pl)
	{
		char line[1024], writing[128];
		long long_number;
		Vector3 wektor;
		quaternion kw;
		float float_number;
		while (fgets(line, 1024, pl))
		{
			sscanf(line, "%s", &writing);

			if (strcmp(writing, "<mapa>") == 0)
				mode_reading_map = 1;

			if (mode_reading_map)
			{
				if (strcmp(writing, "<liczba_wierszy") == 0)
				{
					sscanf(line, "%s %d ", &writing, &long_number);
					this->number_of_rows = long_number;
				}
				else if (strcmp(writing, "<liczba_kolumn") == 0)
				{
					sscanf(line, "%s %d ", &writing, &long_number);
					this->number_of_columns = long_number;
				}
				else if (strcmp(writing, "<wiersz_punktow") == 0)
				{
					mode_reading_row = 1;
					sscanf(line, "%s %d ", &writing, &long_number);
					nr_of_row_point = long_number;
					nr_of_column_point = 0;
				}
				else if (strcmp(writing, "</mapa>") == 0)
					mode_reading_map = 0;

				if (mode_reading_row)
				{
					if (strcmp(writing, "<w") == 0)
					{
						sscanf(line, "%s %f ", &writing, &float_number);
						height_map[nr_of_row_point][nr_of_column_point] = float_number;
						nr_of_column_point++;
					}
					else if (strcmp(writing, "</wiersz_punktow>") == 0)
						mode_reading_row = 0;
				}
			}

			if ((this->number_of_rows > 0) && (this->number_of_columns > 0) && (height_map == NULL))
			{
				height_map = new float* [number_of_rows * 2 + 1];
				for (long i = 0; i < number_of_rows * 2 + 1; i++) {
					height_map[i] = new float[number_of_columns + 1];
					for (long j = 0; j < number_of_columns + 1; j++) height_map[i][j] = 0;
				}
			}
		}
		fclose(pl);
	}
	else return 0;

	return 1;
}

void Environment::Draw()
{
	glCallList(EnvironmentMap);
}