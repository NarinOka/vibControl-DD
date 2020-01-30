// 07_MARL-boids by Okazaki
// Multi-Agent Reinforcement Learning for vibration control
// 2019.11.14 first mounted
// 2019.11.21 vibration controled by single agent without learning
// 2019.11.26 vibration controled by multiple agent without learning and moving
// 2019.11.26 change sensors to acceleration sensors
// 2019.12.16 enable to set whether use OpenGL display or not

#include "GLfunc.h"
#include "Calculation.h"
#include "definedHeaders.h"
#include "csv_parser.hpp"

#pragma warning(disable : 4996)	// to use localtime

//-------User Control Section-------
#define BETA 1.0 / 6.0	//linear acceleration method (β=1/6 for Newmark-beta)
#define FILEOPEN false	//if you want to output csv, turn this into "true"
#define LEARN false	//if you need learning part, turn this into "true"
#define CHECK_VIB true //output csv of all DoF
#define SHOW_GL false	//if you want to check vib visually, turn this into "true"
#define SHOW_WIN2 false	//window for "withoutCtrl" これをtureにするなら，aaとaa_invのサイズを工夫するべき
const float maxSimTime = 10;	//max simulation time
const float controlStartTime = 2.0;	//control start time
// const float controlStartTime = maxSimTime + 1.0;	//without control
const float learnStartTime = 2.0;	//learn start time

//-------bitmap出力用-------
bool _Bitmap = false;
gl_screenshot gs; // bmpファイルの出力
int tn1 = 0;
int tn2 = 0;
std::string date;
std::string time_now;
std::ostringstream bmpFolderPath;
//これをtrueにしてbmpを得る→AVImakerを使って動画aviを作る
//画像を保存しようとするとかなり動作が重くなるので，普段はfalseにしておくこと．
//その時に，対応するフォルダを作っておくこと．(そうしないと上書きされるor画像保存されない)
// http://www.natural-science.or.jp/article/20091111213113.php


//-------Variables for calculation (global variables)-------
Eigen::VectorXf f1(DoF), f2(DoF);	// 1: window1(with control) 2: window2(without control)
Eigen::MatrixXf aa(DoF, DoF), aa_inv(DoF, DoF);


float t = 0.0;    //時刻[s]
float dt = 0.005; //分解能[s]
int simSteps = 0;

int WinID[2] = { 0 };   //ウィンドウのID
int winNum = 0;       //今処理しているウィンドウの数字

ofstream allDisplacement;
ofstream actHistory;
ofstream modeChange_byAct;
// learn
ofstream learnError;
ofstream networkParams;
ofstream learningConditions;
ofstream result;


std::ostringstream learnFolderPath;
std::ostringstream dispFolderPath;


Plant withCtrl(MASS, DAMPER, SPRING);	//initialize instance "withCtrl"
Plant withoutCtrl(MASS, DAMPER, SPRING);	//initialize instance "withoutCtrl"
/*動的ver*/
//SensorUnit **sensorUnits = new SensorUnit*[sensorUnitNum];	//sensor units(センサユニット群)
//ActUnit **actUnits = new ActUnit*[actUnitNum];	//actuator units(アクチュエータユニット群)
SensorUnit sensorUnits[sensorUnitNum];
ActUnit actUnits[actUnitNum];

bool actUnitIsHere[DoF] = { false };
int workingActNum = 0;	//withCtrl(Plant)に付いている総アクチュエータユニットの数
int attachedActNum[DoF] = { 0 }; // attached actuator unit number at each DoF
bool initBiggest = false;

/*** for learning ***/
int learnCount = 0;
float Ep[waveNum] = { 0.0 }; // pattern error
float Et = 0.0; // total error
int inputIndex = 0;	//入力層ユニットの番号

//出力用
vector<vector<float>> input_NN;	//sensorUnits[0].inputNumが未定であるため
float output_NN[waveNum][DoF] = { 0.0 };
float teach_NN[waveNum][DoF] = { 0.0 };

//------- Prototype Declaration -------
void calc_setup();
void reinitVariables();
void calc_vib();
void display();
void timer(int num);
void initSensorUnits(Plant& p);
void initActUnits();
void vibControl_byUnits(ActUnit(&actUnits)[actUnitNum], SensorUnit(&sensorUnits)[sensorUnitNum]);
bool dispofAllDoF(Plant& p);
void fileOpen(bool fopen);
void fileClose();
void outBMP(bool bmp);
void resize(int w, int h);
void keyboard(unsigned char key, int x, int y);
void getDayTime();
void eachStep();
void eachStep_display();
void eachStep_timer();
void learnPos_bySensor(int wave_now);
void calc_error(SensorUnit(&sensorUnits)[sensorUnitNum]);
void calc_forwardNN(SensorUnit(&sensorUnits)[sensorUnitNum]);

/****************Caluculate Vibration****************/
// Newmark-β法を使うための行列の準備
void calc_setup()
{
	withCtrl.set_simFreq();
	withoutCtrl.simFreq = withCtrl.simFreq;

	/* initialization of variables for newmark-beta */
	f1 = Eigen::VectorXf::Zero(DoF);
	f2 = Eigen::VectorXf::Zero(DoF);
	aa = Eigen::MatrixXf::Zero(DoF, DoF);
	aa_inv = Eigen::MatrixXf::Zero(DoF, DoF);

	aa = withCtrl.M + dt * withCtrl.C / 2 + BETA * dt * dt * withCtrl.K; // a1,a2を求めるときに必要
	aa_inv = aa.inverse();

}


void reinitVariables()
{	// call this func when to restart calculation from the beginning

	t = 0.0;
	// dt = 0.005;
	simSteps = 0;
	workingActNum = 0;	
	for(int i= 0;i<DoF;i++)
		attachedActNum[i] = 0;

	// plant
	withCtrl.y = 0.0;
	withCtrl.vy = 0.0;
	withCtrl.ay = 0.0;

	if (useDD) {
		withCtrl.x = Eigen::VectorXf::Zero(DoF + workingActNum);
		withCtrl.vel = Eigen::VectorXf::Zero(DoF + workingActNum);
		withCtrl.accel = Eigen::VectorXf::Zero(DoF + workingActNum);
	}
	else {
		withCtrl.x = Eigen::VectorXf::Zero(DoF);
		withCtrl.vel = Eigen::VectorXf::Zero(DoF);
		withCtrl.accel = Eigen::VectorXf::Zero(DoF);
	}

	withCtrl.biggestDisps = Eigen::VectorXf::Zero(DoF);

	withoutCtrl.y = 0.0;
	withoutCtrl.vy = 0.0;
	withoutCtrl.ay = 0.0;	

	withoutCtrl.x = Eigen::VectorXf::Zero(DoF);
	withoutCtrl.vel = Eigen::VectorXf::Zero(DoF);
	withoutCtrl.accel = Eigen::VectorXf::Zero(DoF);
	withoutCtrl.biggestDisps = Eigen::VectorXf::Zero(DoF);

	// sensor units
	for (SensorUnit &s : sensorUnits) {
		s.countSteps = 0;
		s.biggestDisp = 0.0;
		s.biggestDisp_past = 0.0;
		s.acceleration = 0.0;
		s.velocity = 0.0;
		s.displacement = 0.0;
	}
}


//各時刻における位置の算出(Newmark-beta method)
void calc_vib()
{
		Eigen::VectorXf x_past(DoF), v_past(DoF), a_past(DoF),
			f_past(DoF); // 1step前(現在)の値


	if (winNum == 0)
	{ // with control
		if (useDD) {
			x_past.resize(DoF + workingActNum);
			v_past.resize(DoF + workingActNum);
			a_past.resize(DoF + workingActNum);
			f_past.resize(DoF + workingActNum);
		}

	  // External force	//external force from displacement excitation added to
	  // DoF No.1
		f1[0] += (withCtrl.k[0] * withCtrl.y + withCtrl.c[0] * withCtrl.vy);

		x_past = withCtrl.x;
		v_past = withCtrl.vel;
		a_past = withCtrl.accel;

		withCtrl.accel =
			aa_inv * (f1 - withCtrl.C * (v_past + a_past * dt / 2.0) -
				withCtrl.K * (x_past + dt * v_past + (0.5 - BETA) * dt * dt * a_past));

		withCtrl.vel = v_past + 0.5 * (withCtrl.accel + a_past) * dt;

		withCtrl.x = x_past + v_past * dt + (0.5 - BETA) * a_past * dt * dt +
			BETA * withCtrl.accel * dt * dt;

		//振幅(モード)の記録
		//if (t > controlStartTime + 1.0 && t < controlStartTime + 2.0) //振動制御している時間内
		//{
		//	//対象とする振動モードと同じ番号の自由度の振幅が最大となるときの
		//	//各自由度の振幅を得る
		//	if (withCtrl.modeAmp[VIBMODE - 1] < withCtrl.x[VIBMODE - 1]) {
		//		for (int i = 0; i < DoF; i++)
		//			withCtrl.modeAmp[i] = withCtrl.x[i];
		//	}
		//}

		withCtrl.getBiggestDisps();
		// cout << withCtrl.biggestDisps << endl << endl;



		/************* for sensor units *************/
		
		// sensors
		for (int i = 0; i < sensorUnitNum; i++) {
			sensorUnits[i].readAcceleration(withCtrl);
			sensorUnits[i].cal_velocity(dt);
			sensorUnits[i].cal_displacement(dt);
			//cout << sensorUnits[i].displacement << endl;
		}
		// get biggest displacement ≒ amplitude
		for (SensorUnit &s : sensorUnits) {
			s.getBiggestDisp();	//s0はいらないかも
			//cout << s.biggestDisp << endl;
		}
		//cout << endl;

		//sensorUnits[actUnits[0].currentPos - 1].readDisplacement(withCtrl);

		/************* for actuator units *************/
		//for (int i = 0; i < actUnitNum; i++) {
		//	//actUnits[i].getEachBiggestDisp(sensorUnits);
		//	int atBiggest_temp = actUnits[i].judgeBiggestPos(sensorUnits);	//in_dispを得る
		//}
		//
		//int attachedPos;
		//if (t > controlStartTime) {
		//	// actUnits[i].moveTo(actUnits[i].judgeBiggestPos(sensorUnits));
		//	for (workingActNum; workingActNum < actUnitNum; workingActNum++) {
		//		//if (workingActNum < actUnitNum) {
		//		if (simSteps > addUnitCoef * getAmp_steps * (workingActNum + 1)) {
		//			//cout << simSteps << "\t" << addUnitCoef * getAmp_steps * (workingActNum + 1) << endl;
		//
		//			if (workingActNum < DoF) {
		//				actUnits[workingActNum].moveTo(actUnits[workingActNum].
		//					in_disp[workingActNum].first + 1);
		//				attachedPos = actUnits[workingActNum].in_disp[workingActNum].first;
		//				actUnitIsHere[workActUnitNum] = true;
		//			}
		//			else {
		//				actUnits[workingActNum].moveTo(actUnits[workingActNum].
		//					in_disp[workingActNum - DoF].first + 1);	//とりあえず
		//				attachedPos = actUnits[workingActNum].in_disp[workingActNum - DoF].first;
		//			}
		//			attachedActNum[attachedPos] += 1;
		//			actUnits[workingActNum].whatNumAtDoF = attachedActNum[attachedPos];
		//
		//			cout << "actuator unit " << workingActNum + 1 << " moved to position: "
		//				<< attachedPos + 1 << endl;
		//			cout << "attachedActNum " << attachedActNum[attachedPos] << endl;
		//			cout << actUnits[workingActNum].biggestDisps << endl;
		//
		//			/*initialization of biggestDisps in sensor*/
		//			for (int i = 0; i < DoF; i++) {
		//				if (sensorUnits[i].checkSteps())
		//					sensorUnits[i].biggestDisp = 0.0;
		//			}
		//
		//		}
		//	}
		//}



	}
	else
	{ // without control
	  // External force	//external force from displacement excitation added to
	  // DoF No.1
		f2[0] += (withoutCtrl.k[0] * withoutCtrl.y + withoutCtrl.c[0] * withoutCtrl.vy);

		x_past = withoutCtrl.x;
		v_past = withoutCtrl.vel;
		a_past = withoutCtrl.accel;

		withoutCtrl.accel =
			aa_inv * (f2 - withoutCtrl.C * (v_past + a_past * dt / 2.0) -
				withoutCtrl.K * (x_past + dt * v_past + (0.5 - BETA) * dt * dt * a_past));

		withoutCtrl.vel = v_past + 0.5 * (withoutCtrl.accel + a_past) * dt;

		withoutCtrl.x = x_past + v_past * dt + (0.5 - BETA) * a_past * dt * dt +
			BETA * withoutCtrl.accel * dt * dt;

		//振幅(モード)の記録
		//if (t > controlStartTime + 1.0 && t < controlStartTime + 2.0) //振動制御している時間内
		//{
		//	//対象とする振動モードと同じ番号の自由度の振幅が最大となるときの
		//	//各自由度の振幅を得る
		//	if (withoutCtrl.modeAmp[VIBMODE - 1] < withoutCtrl.x[VIBMODE - 1]) {
		//		for (int i = 0; i < DoF; i++)
		//			withoutCtrl.modeAmp[i] = withoutCtrl.x[i];
		//	}
		//}
	}
}

//---------------図形の描画はここで行う---------------
void display()
{
	glClear(GL_COLOR_BUFFER_BIT); //ウィンドウの塗りつぶし

	int i;

	if (glutGetWindow() == WinID[0])
	{
		winNum = 0;
	}
	else
	{
		winNum = 1;
	}


	// -----output file-----
	outBMP(_Bitmap);
	// ----- -----
	eachStep_display();

	//質点間を結ぶ線の描画 //縦長の線
	glLineWidth(30);
	glBegin(GL_LINE_STRIP);
	glColor3d(76 / 255.0, 74 / 255.0, 74 / 255.0);
	// glColor3d(255 / 255.0, 255 / 255.0, 255 / 255.0);
	glVertex2d(0.3, -0.9);
	for (i = 1; i < (DoF + 1); i++)
	{
		if (winNum == 0)
		{
			glVertex2d(0.3 + withCtrl.x[i - 1], -0.9 + i * length);
		}
		else
		{
			glVertex2d(0.3 + withoutCtrl.x[i - 1], -0.9 + i * length);
		}
	}
	if (winNum == 0)
	{
		glVertex2d(0.3 + withCtrl.x[DoF - 1], -0.9 + DoF * length + 0.05);
	}
	else
	{
		glVertex2d(0.3 + withoutCtrl.x[DoF - 1], -0.9 + DoF * length + 0.05);
	}
	glEnd();


	glBegin(GL_LINE_STRIP);
	glColor3d(76 / 255.0, 74 / 255.0, 74 / 255.0);
	// glColor3d(255 / 255.0, 255 / 255.0, 255 / 255.0);
	glVertex2d(-0.3, -0.9);
	for (i = 1; i < (DoF + 1); i++)
	{
		if (winNum == 0)
		{
			glVertex2d(-0.3 + withCtrl.x[i - 1], -0.9 + i * length);
		}
		else
		{
			glVertex2d(-0.3 + withoutCtrl.x[i - 1], -0.9 + i * length);
		}
	}
	if (winNum == 0)
	{
		glVertex2d(-0.3 + withCtrl.x[DoF - 1], -0.9 + DoF * length + 0.05);
	}
	else
	{
		glVertex2d(-0.3 + withoutCtrl.x[DoF - 1], -0.9 + DoF * length + 0.05);
	}
	glEnd();


	glBegin(GL_QUADS); // 4点をまとめて塗りつぶすモード
					   //土台の描画
	glColor3d(173 / 255.0, 73 / 255.0, 30 / 255.0);
	glVertex2d(-0.9, -0.95);
	glVertex2d(0.9, -0.95);
	glVertex2d(0.9, -0.85);
	glVertex2d(-0.9, -0.85);

	//各質点の初期位置の表示
	// glColor3d(183 / 255.0, 99 / 255.0, 30 / 255.0);
	// for (i = 1; i < (winNum + 1); i++){
	//	glVertex2d(-0.3, -0.95 + i*length);
	//	glVertex2d(0.3, -0.95 + i*length);
	//	glVertex2d(0.3, -0.85 + i*length);
	//	glVertex2d(-0.3, -0.85 + i*length);
	//}

	//各質点の描画
	glColor3d(37 / 255.0, 20 / 255.0, 6 / 255.0);
	for (i = 1; i < (DoF + 1); i++)
	{
		if (winNum == 0)
		{
			glVertex2d(-0.3 + withCtrl.x[i - 1], -0.95 + i * length);
			glVertex2d(0.3 + withCtrl.x[i - 1], -0.95 + i * length);
			glVertex2d(0.3 + withCtrl.x[i - 1], -0.85 + i * length);
			glVertex2d(-0.3 + withCtrl.x[i - 1], -0.85 + i * length);

		}
		else
		{
			glVertex2d(-0.3 + withoutCtrl.x[i - 1], -0.95 + i * length);
			glVertex2d(0.3 + withoutCtrl.x[i - 1], -0.95 + i * length);
			glVertex2d(0.3 + withoutCtrl.x[i - 1], -0.85 + i * length);
			glVertex2d(-0.3 + withoutCtrl.x[i - 1], -0.85 + i * length);
		}
	}
	glEnd();

	//sensor units
	glColor4f(173.0 / 255.0, 180.0 / 255.0, 216.0 / 255.0, 1.0f);
	if (winNum == 0)
	{
		drawCircle(0.08, 50, -0.40, -0.90);	// sensor 0
		for (int i = 1; i < sensorUnitNum; i++) {
			drawCircle(0.08, 50,
				-0.35 + withCtrl.x[i - 1], -0.90 + i * length);
		}
	}
	//act unit
	for (int i = 0; i < actUnitNum; i++) {
		if (winNum == 0) {

			if (actUnits[i].currentPos == 0) {	//at ground
				// 枠線
				glColor4f(0.0f, 255.0 / 255.0, 0.0 / 255.0, 1.0f);
				drawCircle(0.09, 50, 0.5 + i * 0.1, -0.90);
				// 中身
				glColor4f(0.0f, 91.0 / 255.0, 172.0 / 255.0, 1.0f);
				drawCircle(0.08, 50, 0.5 + i * 0.1, -0.90);
			}
			else {
				// 枠線
				glColor4f(0.0f, 255.0 / 255.0, 0.0 / 255.0, 1.0f);
				drawCircle(0.09, 50,
					0.35 + withCtrl.x[actUnits[i].currentPos - 1]
					+ (actUnits[i].whatNumAtDoF - 1) * 0.1,
					-0.90 + (actUnits[i].currentPos) * length);
				// 中身
				glColor4f(0.0f, 91.0 / 255.0, 172.0 / 255.0, 1.0f);
				drawCircle(0.08, 50,
					0.35 + withCtrl.x[actUnits[i].currentPos - 1]
					+ (actUnits[i].whatNumAtDoF - 1) * 0.1,
					-0.90 + (actUnits[i].currentPos) * length);

			}
		}
	}

	glFlush(); //実行されていないOpenGLの命令をすべて実行する
}


/**************** Vibration Control Units ****************/
//initialize sensor units
//クラス配列の初期化参考:https://blog.livlea.com/entry/2018/08/cpp-class-init
void initSensorUnits(Plant& p) //付加される制御対象を引数とする
{
	/*動的ver*/
	////std::ostringstream actUnitName;
	//for (int i = 0; i < sensorUnitNum; i++) {
	//	//actUnitName << "actUnit" << i + 1;
	//	sensorUnits[i] = new SensorUnit(i, p);
	//}

	for (int i = 0; i < sensorUnitNum; i++) {
		sensorUnits[i] = SensorUnit(i, p);
	}
}

//initialize actuator units
void initActUnits()
{
	for (int i = 0; i < actUnitNum; i++) {
		actUnits[i] = ActUnit(i + 1);
	}
}


// Vibration control by units
void vibControl_byUnits(ActUnit (&actUnits)[actUnitNum], SensorUnit (&sensorUnits)[sensorUnitNum])
{
	/*一気に動かす用*/
	//for (int i = 0; i < actUnitNum; i++) {
	//	// actUnits[i].moveTo(actUnits[i].actID);	// test
	//
	//	if (actUnits[i].currentPos == 0) {
	//		;	// To avoid "assertion failed"
	//	}
	//	else {
	//	int pos = actUnits[i].currentPos;
	//	//f1[pos - 1] += actUnits[i].outputForce(sensorUnits[pos - 1]);
	//	f1[pos - 1] += - actUnits[i].dampingCoef * withCtrl.vel[pos - 1];
	//	}
	//}


	//learning後メモ
	/*
	学習で用いた振動数のうち，どれか一つについてのAユニットの挙動を見る．
	学習によって調整されたパラメータを用いて出力された値のうち，
	最大のところにまず移動する．
	その後はSユニットの出力を逐次見てAユニットがなくなるまで
	振幅が最大になっている箇所へ移動する．
	*/
	//actUnits[0].currentPos = 2;
	//int pos = actUnits[0].currentPos;
	//f1[pos - 1] += -actUnits[0].dampingCoef * withCtrl.vel[pos - 1];

	// choose position
	int pos = 0;

	if (workingActNum < actUnitNum) {
		if(t > controlStartTime + workingActNum * controlStartTime){
			if (workingActNum == 0) { //最初の一体はネットワークの出力が一番大きいところに移動
				calc_forwardNN(sensorUnits);
				sensorUnits[0].sort_NNoutputs(sensorUnits);
				pos = sensorUnits[0].sortedNNoutputs[0].first;//一番大きいところ

				actUnits[workingActNum].moveTo(pos);

				// file output
				for (int i = DoF; i > 0; i--) {
					//modeChange_byAct << i << "," << sensorUnits[i].output << endl;	//normalized
					modeChange_byAct << i << "," << sensorUnits[i].biggestDisp << endl;	//original amp
				}
			}
			else {//その後，センサーの読み取り値をそのまま利用
				sensorUnits[0].sort_biggestDisps(sensorUnits);
				pos = sensorUnits[0].sortedBiggestDisps[0].first;//一番大きいところ

				actUnits[workingActNum].moveTo(pos);

				// file output
				for (int i = DoF; i > 0; i--) {
					sensorUnits[i].teach_sysID(sensorUnits[0]);// to get normalized biggest disp

					//modeChange_byAct << i << "," << sensorUnits[i].normalizedBiggestDisp << endl;	// normalized
					modeChange_byAct << i << "," << sensorUnits[i].biggestDisp << endl;	// original disp
				}
			}

				modeChange_byAct << "0,0" << endl;
				modeChange_byAct << endl;


			// file output
			actHistory << t << "," << pos << "," << actUnits[workingActNum].dampingCoef << endl;

			cout << "### actuator unit " << workingActNum + 1 << " moved to " << pos << " ###" << endl;
			workingActNum++;
			initBiggest = true;

			// for display
			attachedActNum[pos - 1]++;
			actUnits[workingActNum].whatNumAtDoF = attachedActNum[pos - 1];

			// attach dynamic damper
			if (useDD) {
				actUnits[workingActNum - 1].attachDD(withCtrl, f1);

				// recalculate matrix
				aa = withCtrl.M + dt * withCtrl.C / 2 + BETA * dt * dt * withCtrl.K; 
				aa_inv = aa.inverse();

				cout << "M-matrix is:\n" << withCtrl.M << endl;
				cout << "C-matrix is:\n" << withCtrl.C << endl;
				cout << "K-matrix is:\n" << withCtrl.K << endl;
				cout << "aa-matrix is:\n" << aa << endl;
				cout << "aa_inv-matrix is:\n" << aa_inv << endl << endl;
			}

		}

	}
	if (t > controlStartTime + (workingActNum - 1) * controlStartTime + 0.4 && initBiggest) {
		//より正確にモードが得られるように
		for (int i = 1; i < sensorUnitNum; i++) {
			sensorUnits[i].biggestDisp = 0.0;	//initialization for next step
		}
		initBiggest = false;
	}

	// additional damping force
	if (!useDD) {
		for (int i = 0; i < actUnitNum; i++) {
			if (actUnits[i].currentPos != 0) {
				f1[actUnits[i].currentPos - 1] += -actUnits[i].dampingCoef * withCtrl.vel[actUnits[i].currentPos - 1];
			}
		}
	}

}


// judgement for simulation termination
bool dispofAllDoF(Plant& p)
{
	bool finish = false;

	// if ((p.x.array().abs() < p.MinofMax_vector.array()).all()) {
	// 	// cout << p.x.array().abs() << endl << endl;
	// 	finish = true;
	// }

	//termination condition regarding displacements
	//if ((actUnits[0].biggestDisps.array() < p.MinofMax_vector.array()).all()) {
	//	 cout << actUnits[0].biggestDisps << endl << endl;
	//	finish = true;
	//}

	if (p.vibTimesChanged) {
		if ((p.biggestDisps.array() < p.MinofMax_vector.array()).all()) {
			finish = true;
		}
		//cout << p.biggestDisps << endl << endl;
		p.biggestDisps = Eigen::VectorXf::Zero(DoF);
		//cout << p.biggestDisps << endl << endl;
	}

	return finish;

}


/**************** related to FileIO Functions ****************/
void fileOpen(bool fopen)
{
	//openしなければfileに書き込まれない
	std::ostringstream oss_vibmode;
	oss_vibmode << VIBMODE;
	std::ostringstream oss_DoF;
	oss_DoF << DoF;
	std::ostringstream oss_actPos[actUnitNum];
	for (int i = 0; i < actUnitNum; i++) {
		oss_actPos[i] << actUnits[i].currentPos;
	}
	std::ostringstream oss_freq;
	oss_freq << withCtrl.simFreq;

	if (fopen) {
		if (LEARN) {
			learnError.open(learnFolderPath.str() + "\\" + time_now + "_error" + ".csv");
			networkParams.open(learnFolderPath.str() + "\\" + time_now + "_params" + ".csv");
			learningConditions.open(learnFolderPath.str() + "\\" + time_now + "_lc" + ".csv");
			result.open(learnFolderPath.str() + "\\" + time_now + "_result" + ".csv");
		}
		if (CHECK_VIB) {
			//allDisplacement.open("./Data/allDisplacement/disps_mode"
			//	+ oss_vibmode.str() + "_DoF" + oss_DoF.str() 
			//	+ "_" + time_now + ".csv");
			allDisplacement.open(dispFolderPath.str() + "\\" + time_now + "_freq" + oss_freq.str() + ".csv");
			actHistory.open(dispFolderPath.str() + "\\" + time_now + "_freq" + oss_freq.str() + "_actHist" + ".csv");
			modeChange_byAct.open(dispFolderPath.str() + "\\" + time_now + "_freq" + oss_freq.str() + "_modeChange" + ".csv");
		}
			//temp
			//sensorANDplant.open("./Data/sensorANDplant/mode"
			//	+ oss_vibmode.str() + "_DoF" + oss_DoF.str() 
			//	+ "_" + time_now + ".csv");
			//sensorValues.open("./Data/sensorValues/disps_mode"
			//	+ oss_vibmode.str() + "_DoF" + oss_DoF.str() 
			//	+ "_" + time_now + ".csv");
			//time_displacement_force.open("./Data/time_displacement_force/tdf_vel_mode"
			//	+ oss_vibmode.str() + "_actPos" + oss_actPos.str() + ".csv");
			//allDisplacement.open("./Data/allDisplacement/disps_vel_mode"
			//	+ oss_vibmode.str() + "_actPos" + oss_actPos.str() + ".csv");			//controlled.open("./Data/controlled/mode" + oss_vibmode.str() + ".csv");
			//vibmodes.open("./Data/vibmodes/mode" + oss_vibmode.str() + ".csv");


		/* Headers */
		//--- allDisplacement
		allDisplacement << ",Basement";
		for (int i = 0; i < DoF; i++) {
			allDisplacement << ",Mass " << i + 1;
		}
		allDisplacement << endl;
		//--- actHistory
		actHistory << "Simulation time [s],Position,Damping coefficient" << endl;
		//--- modeChange_byAct
		modeChange_byAct << "Position,Amplitude" << endl;
		//--- learnError
		learnError << "Learning times,Error" << endl;
		//--- networkParams
		networkParams << "Learning times" ;
		for (int in = 0; in < sensorUnits[0].inputNum; in++) {
			for (int dof = 0; dof < DoF; dof++) {
				networkParams << ",wei_" << in + 1 << "-" << dof + 1;
			}
		}
		for (int j = 0; j < DoF; j++) {
			networkParams << ",bias_" << j + 1;
		}
		networkParams << endl;
		//--- learningConditions
		learningConditions << "waveNum," << waveNum << endl;
		learningConditions << "frequency [Hz]";
		for (int i = 0; i < waveNum; i++)
			learningConditions << ',' << freqSet[i];
		learningConditions << endl;
		learningConditions << "input wave range [s]," << sensorUnits[0].inputWaveRange << endl;
		learningConditions << "input wave dt [s]," << sensorUnits[0].input_dt << endl;
		learningConditions << "input unit num [-]," << sensorUnits[0].inputNum << endl;
		learningConditions << "DoF," << DoF << endl;
		learningConditions << "Learning coefficient," << alpha << endl;	//wとb両方同じ値
		learningConditions << "ini w and b(rand)," << w_ini << endl;	//wとb両方同じ値
		learningConditions << "a_sigmoid," << aSig << endl;	
		learningConditions << "sim resolution (=dt)[s]," << dt << endl;	

		//--- result
		result << "Position,output,teach" << endl;


	}
}

void fileClose()
{
	allDisplacement.close();
	actHistory.close();
	modeChange_byAct.close();
	learnError.close();
	networkParams.close();
	learningConditions.close();
	result.close();
}

void outBMP(bool bmp)
{
	if (bmp) {
		if (simSteps % 5 == 0) { // save data amount
			if (winNum == 0)
			{
				std::ostringstream fname;
				int tt = tn1 + 10000;
				fname << bmpFolderPath.str() << "/" << tt << ".bmp"; //出力ファイル名
				//※フォルダが同じ階層になかったらコンソールにエラー出る
				//※めちゃくちゃ重くなる．
				string name = fname.str();
				gs.screenshot(name.c_str(), 24);
				tn1++;
			}
			else
			{
				// ostringstream fname;
				// int tt = tn2 + 10000;
				// fname << "Data/bitmap/window2-20190930-fn5/" << tt << ".bmp"; //出力ファイル名
				// string name = fname.str();
				// gs.screenshot(name.c_str(), 24);
				// tn2++;
			}
		}
	}
}


/**************** functions for GL ****************/
void timer(int num)
{
	//if (t >= maxSimTime) exit(0);	//terminate program
	if (t >= maxSimTime
		|| (dispofAllDoF(withCtrl) && t > controlStartTime + 0.5)
		)	
		// terminate conditions except Esc key
	{
		cout << "---termination conditions satisfied---" << endl;
		glutLeaveMainLoop();	//leave infinite glutMainLoop
	}

	glutTimerFunc(10, timer, 0);

	eachStep_timer();

	//cout << t << endl;

	glutSetWindow(WinID[0]);
	glutPostRedisplay(); //再描画の必要を訴える

	if (SHOW_WIN2) {
		glutSetWindow(WinID[1]);
		glutPostRedisplay();
	}


}


void resize(int w, int h)
{
	// https://www2.akita-nct.ac.jp/take/note/info2e_pre/pdf/info2e17-01.pdf
	glLoadIdentity();// 変換行列を単位行列に設定

	// 描画するワールド座標系の範囲を指定
	//gluOrtho2D(-1.0, 1.0, -1.0, h / 500.0);
	gluOrtho2D(-1.0, 1.0, -1.0, 0.1 - 0.85 + length*DoF);
	//gluOrtho2D(-1.0, 1.0, -1.0, extra_to_1);

	glViewport(0, 0, w, h); // ウィンドウの描画領域を指定

}


/* キーボードからウィンドウを閉じる設定 */
void keyboard(unsigned char key, int x, int y)
{	// http://web.wakayama-u.ac.jp/~wuhy/GSS/02.html#2.2
	// windowを選択した状態でなければならない
	switch (key) {
	case '\33': /* \33 は ESC の ASCII コード (8進数で表示）*/
		glutLeaveMainLoop();
	default:
		break;
	}
}


// get stringstream of year-month-day
void getDayTime()
{
	// http://rinov.sakura.ne.jp/wp/cpp-date
	//現在日時を取得する
	time_t t = time(nullptr);

	//形式を変換する    
	const tm* lt = localtime(&t);

	//dに独自フォーマットになるように連結していく
	std::stringstream d;
	d << "20";
	d << lt->tm_year - 100; //100を引くことで20xxのxxの部分になる
	d << "-";
	if (lt->tm_mon + 1 < 10)
		d << "0";
	d << lt->tm_mon + 1; //月を0からカウントしているため
	d << "-";
	if (lt->tm_mday < 10)
		d << "0";
	d << lt->tm_mday; //そのまま

	std::stringstream time_str;
	if (lt->tm_hour < 10) 
		time_str << "0";
	time_str << lt->tm_hour; 
	time_str << "-";
	if (lt->tm_min < 10)
		time_str << "0";
	time_str << lt->tm_min; 
	time_str << "-";
	if (lt->tm_sec < 10)
		time_str << "0";
	time_str << lt->tm_sec; 

	//date = "20**-**-**" (year-month-day)
	date = d.str();

	//time_now = "**-**-**" (hr-min-sec)
	time_now = time_str.str();

}

void eachStep_display()
{
	/********** in display func **********/

	// -----output file-----
	allDisplacement << t << "," << withCtrl.y;
	for (int i = 0; i < DoF; i++) {
		allDisplacement << "," << withCtrl.x[i];
	}
	allDisplacement << endl;
	// ----- -----


	withCtrl.update_external(t);
	withoutCtrl.update_external(t);


	//なぜ初期化するか⇒次の関数(vibControlとcalc_vib)で
	//計算した外力の値を+=で代入する前に0としておくため．
	if(useDD)
		f1 = Eigen::VectorXf::Zero(DoF + workingActNum);
	else
		f1 = Eigen::VectorXf::Zero(DoF);

	f2 = Eigen::VectorXf::Zero(DoF);

	// for network calculation
	if (inputIndex < sensorUnits[0].inputNum) {
		if (t > inputIndex * sensorUnits[0].input_dt) {
			sensorUnits[0].setInputValues(inputIndex);
			inputIndex++;
		}
	}

	if (t >= controlStartTime)
		vibControl_byUnits(actUnits, sensorUnits);	//ここは変える必要がある

	//calculation of state quantities of plant and sensor units
	calc_vib();

}

void eachStep_timer()
{

	/********** in timer func **********/
	t += dt; // update time
	simSteps++;
	if (simSteps % 10000 == 0) cout << simSteps << " simulation steps" << endl;

	withCtrl.cal_vibTimes(t);
	withoutCtrl.cal_vibTimes(t);

	for (SensorUnit &s : sensorUnits) {
		s.countSteps++;
	}

}

void eachStep() 
{
	eachStep_display();
	eachStep_timer();
}


/*** for learning ***/


// 2019.12.28 先生に報告
void learnPos_bySensor(int wave_now)
{
	sensorUnits[0].calc_inXwei();	//忘れてた！

	sensorUnits[0].sort_biggestDisps(sensorUnits);
	input_NN[wave_now] = sensorUnits[0].inputValues;

	for (int i = 1; i < sensorUnitNum; i++) {
		/*forward calculation*/
		sensorUnits[i].calc_output(sensorUnits[0]);
		/*update parameters*/
		sensorUnits[i].teach_sysID(sensorUnits[0]);
		sensorUnits[i].calc_dEdb_sensor();
		sensorUnits[i].updateBias_sensor();

		// for file output
		output_NN[wave_now][i - 1] = sensorUnits[i].output;	// あとでinput_NN使って計算しなおす
		teach_NN[wave_now][i - 1] = sensorUnits[i].normalizedBiggestDisp;

		//cout << sensorUnits[i].biggestDisp << endl;
	}
	//cout << endl;

	sensorUnits[0].updateWeight_sensor(sensorUnits);

	calc_error(sensorUnits);

	learnCount++;

		learnError << learnCount << "," << Et << endl;

	// -----output files-----
	if (learnCount == 1 || learnCount % 50 == 0) {
		// ----- -----
		networkParams << learnCount;
		for (int in = 0; in < sensorUnits[0].inputNum; in++) {
			for (int dof = 0; dof < DoF; dof++) {
				networkParams << "," << sensorUnits[0].weights_s0[in][dof];
			}
		}
		for (int i = 0; i < DoF; i++) {
			// actUnitsから，学習ネットワークにユニットをランダムに選ぶのならばここは注意
			networkParams << "," << sensorUnits[i + 1].bias;
		}
		networkParams << endl;
		// ----- -----
	}


	if (learnCount % 200 == 0) {
		cout << "*************** " << learnCount << " times learned ***************" << endl;
		cout << "current freq[Hz] = " << freqSet[wave_now] << ", Et = " << Et << endl;
	}

}

void calc_error(SensorUnit(&sensorUnits)[sensorUnitNum]) 
{// 引数いらない
	Et = 0.0;
	for (int p = 0; p < waveNum; p++) {
		Ep[p] = 0.0;
		for (int out = 0; out < DoF; out++) {
			//Ep[p] += pow(sensorUnits[out + 1].calc_error(), 2);
			Ep[p] += pow(teach_NN[p][out] - output_NN[p][out], 2);
		}
		Ep[p] /= 2.0;
		Et += Ep[p];
	}

}

void calc_forwardNN(SensorUnit (&sensorUnits)[sensorUnitNum])
{
	sensorUnits[0].calc_inXwei();
	sensorUnits[0].sort_biggestDisps(sensorUnits);

	for (int i = 1; i < sensorUnitNum; i++) {
		/*forward calculation*/
		sensorUnits[i].calc_output(sensorUnits[0]);
	}

}


int main(int argc, char *argv[])
{

	getDayTime();

	// folder check for file output 
	if (LEARN && FILEOPEN) {
		learnFolderPath << ".\\Data\\learn\\" << date 
			// << "_" << time_now 
			<< "_DoF" << DoF 
			//<< "_fn" << VIBMODE
			<< "_waveNum" << waveNum;
		CheckTheFolder::checkExistenceOfFolder(learnFolderPath.str());
	}
	if (CHECK_VIB && FILEOPEN) {
		dispFolderPath << ".\\Data\\allDisplacement\\" << date
			// << "_" << time_now 
			<< "_DoF" << DoF
			//<< "_fn" << VIBMODE
			//<< "_waveNum" << waveNum
			;
		CheckTheFolder::checkExistenceOfFolder(dispFolderPath.str());
	}
	if (_Bitmap) {
		bmpFolderPath << ".\\Data\\bitmap\\withCtrl\\"
			<< date << "_DoF" << DoF << "_fn" << VIBMODE;
		CheckTheFolder::checkExistenceOfFolder(bmpFolderPath.str());
	}

	calc_setup();	//←ここでplantのfrequencyの設定を行う．

	/**** initialize units ****/
	initSensorUnits(withCtrl);
	// initSensorUnits(withoutCtrl);	//制御なしの変位を獲得するため(このままだとただinstanceが上書きされる)
	initActUnits();	//withCtrl用

	fileOpen(FILEOPEN);	//open output files
						// sensorUnits[0].inputNumを使用するのでinitSensorUnitsのあと

	// https://stackoverflow.com/questions/15889578/how-can-i-resize-a-2d-vector-of-objects-given-the-width-and-height
	input_NN.resize(waveNum, vector<float>(sensorUnits[0].inputNum)); 

	cout << "M-matrix is:\n" << withCtrl.M << endl;
	cout << "C-matrix is:\n" << withCtrl.C << endl;
	cout << "K-matrix is:\n" << withCtrl.K << endl;
	cout << "aa-matrix is:\n" << aa << endl;
	cout << "aa_inv-matrix is:\n" << aa_inv << endl;
	cout << "natural frequencies [Hz] are:\n" << withCtrl.natural_freq << endl;
	cout << "natural modes are:\n" << withCtrl.natural_modes << endl << endl;
	cout << "vibration period T[s] is :" <<withCtrl.simT << endl;


	if (LEARN) {
		cout << "*********************** START LEARNING ***********************" << endl;
		cout << "Press \" Esc key \" to finish." << endl << endl;

		reinitVariables();
		//waveNum分の要素をもつvectorをつくってあとでshuffle
		vector<int> v(waveNum);
		iota(v.begin(), v.end(), 0); // 0～waveNum-1 までの値を生成

		// 注意
		while (learnCount < LNum) {
			/* learnPos_byAct */

			// eachStep();
			// 
			// if (t > controlStartTime) {
			// 	learnPos_byAct();
			// 
			// }


freChanged:
			/* learnPos_bySensor */
			//まずここにshuffle操作を入れて振動数がランダムに選択されるようにしたい．
			random_device seed_gen;
			mt19937 engine(seed_gen());
			shuffle(v.begin(), v.end(), engine);

			// process of each freq
			for (int i = 0; i < waveNum; i++) {
freChanged2:
				
				//cout << "Freq for learning = " << freqSet[v[i]] << endl;
					withCtrl.simFreq = freqSet[v[i]];	//shuffleした周波数に設定

				//do {//ある程度覚えてから周波数を変えるように変更

					sensorUnits[0].velocity = withCtrl.y_0 * 2 * M_PI * withCtrl.simFreq;	//消したい

					inputIndex = 0;	//入力層ユニットの番号
					while (inputIndex < sensorUnits[0].inputNum) {

						// for keyboard http://www.charatsoft.com/develop/otogema/page/07input/index.html
						if (GetKeyState(VK_ESCAPE) & 0x8000) // escape
							goto finish; // http://www9.plala.or.jp/sgwr-t/c/sec06-6.html

						//要改良

						//if (GetKeyState(VK_RETURN) & 0x8000) { //return(enter)
						//	inputIndex = sensorUnits[0].inputNum - 1;
						//
						//	if (v[i] == waveNum - 1) {
						//		cout << "--- frequency for learning changed ---" << endl;
						//		
						//		goto freChanged;
						//	}
						//	else {
						//		cout << "--- frequency for learning changed " 
						//			<< freqSet[v[i]] << " → " << freqSet[v[i + 1]] << endl;
						//		i++;
						//
						//		goto freChanged2;
						//	}
						//}


						eachStep();

						if (t > inputIndex * sensorUnits[0].input_dt) {
							sensorUnits[0].setInputValues(inputIndex);
							//cout << inputIndex << "	"<< sensorUnits[0].inputValues[inputIndex] << endl;
							inputIndex++;
						}
					}
					//cout << "freq = " << freqSet[v[i]] << endl;

					learnPos_bySensor(v[i]);
					reinitVariables();
					inputIndex = 0;

				//} while (Et > 0.1);	//この値次第で学習回数が大幅に変わるので注意
			}
			if (Et < 0.0001)	break;
		}

	}
	else { // !LEARN
		// get values of weight and bias

		int freqForLearnNum = 3; //仮
		std::vector<float> freqForLearn;

		cout << "reading csv..." << endl;

		// learn時のsimulation conditionを読み取り
		csv_parser lc;
		lc.read("./Data/learn/2019-12-28_DoF5_waveNum3/16-12-25_lc.csv");
		for each(auto row in lc.rows())
		{
			if (row[0] == "input wave range [s]")	sensorUnits[0].inputWaveRange = stoi(row[1]);
			if (row[0] == "input wave dt[s]")	sensorUnits[0].input_dt = stoi(row[1]);
			if (row[0] == "input unit num [-]")	sensorUnits[0].inputNum = stoi(row[1]);
			if (row[0] == "waveNum")	freqForLearnNum = stoi(row[1]);
			if (row[0] == "frequency [Hz]") {
				for (int i = 0; i < freqForLearnNum; i++) {
					freqForLearn.emplace_back(stof(row[i + 1]));
				}
			}
		}
		// もしwithCtrl.simFreqがfreqForLearnになかったらtypeしなおさせたい

		csv_parser parser;
		parser.read("./Data/learn/2019-12-28_DoF5_waveNum3/16-12-25_params2.csv");
		int col = 1;	//weightやbiasの読み取り列数
		//weight
		for (int i = 0; i < sensorUnits[0].inputNum; i++) {
			for (int j = 0; j < sensorUnitNum - 1; j++) {
				sensorUnits[0].weights_s0[i][j] = stof(parser.rows()[0][col]);
				// cout << "weight_" <<i+1<<"_"<<j+1<<"="<< stof(parser.rows()[0][col]) << endl;
				col++;
			}
		}
		//bias
		for (int j = 1; j < sensorUnitNum; j++) {
			sensorUnits[j].bias = stof(parser.rows()[0][col]);
			// cout << "bias = " << stof(parser.rows()[0][col]) << endl;
			col++;
		}

		cout << "----- network parameter setting completed -----" << endl;

	}

	if (!SHOW_GL) { //without display
		reinitVariables();

			while (t <= maxSimTime
			&& !(dispofAllDoF(withCtrl) && t > controlStartTime + 0.5)
			) {

			eachStep();
		}
	}
	else if (SHOW_GL) {
		reinitVariables();

		glutInit(&argc, argv);          // GLUTおよびOpenGL環境を初期化
		glutInitDisplayMode(GLUT_RGBA); //ディスプレイの色をRGBで指定

		// to leave glut main loop
		glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

		//ウィンドウの位置とサイズの指定
	//-------Window for withCtrl----------------------------------------------------------------------------
		glutInitWindowPosition(100, 100);
		//glutInitWindowSize(WIDTH, (int)HEIGHT*DoF / 5);
		//glutInitWindowSize((int)WIDTH*5/DoF*1.5, HEIGHT);	//10 DoFでちょうどよさげ
		glutInitWindowSize(WIDTH, HEIGHT);

		WinID[0] = glutCreateWindow("with control"); //ウィンドウを開く：ウィンドウのタイトルはここで指定する
		glutDisplayFunc(display); //引数は開いたウィンドウ内に描画する関数へのポインタ
		// glClearColor(176 / 255.0, 215 / 255.0, 237 / 255.0, 1.0); //水色
		glutReshapeFunc(resize); //ウィンドウサイズ変更イベント時のコールバック関数の設定
		 /* 入力処理ルーチンの設定 */
		glutKeyboardFunc(keyboard);
		glClearColor(1.0, 1.0, 1.0,	1.0); //ウィンドウを塗りつぶす色の指定（R，G，B，不透明度）
	//-------Window for withoutCtrl----------------------------------------------------------------------------
		if (SHOW_WIN2) {
			glutInitWindowPosition(600, 100);
			//glutInitWindowSize(WIDTH, (int)HEIGHT*DoF / 5);
			glutInitWindowSize(WIDTH, HEIGHT);
			
			WinID[1] = glutCreateWindow("without control"); 
			glutDisplayFunc(display); 
			glutReshapeFunc(resize); 
			  /* 入力処理ルーチンの設定 */
			glutKeyboardFunc(keyboard);
			glClearColor(1.0, 1.0, 1.0, 1.0); 
		}
	//-----------------------------------------------------------------------------------

		glutTimerFunc(10, timer, 0); // 100msec.後にtimer1();関数起動
		glutMainLoop(); //無限ループ：プログラムがイベント待ちになる

	}

finish:
	//to check leaving glutMainLoop
	cout << endl << "*********************** finished! ***********************" << endl << endl;


	//--- learningConditions
	learningConditions << "Learning times," << learnCount << endl;

	//--- result
	for (int wave = 0; wave < waveNum; wave++) {
		sensorUnits[0].inputValues = input_NN[wave];
		sensorUnits[0].calc_inXwei();
		for (int i = 1; i < sensorUnitNum; i++) {
			/*forward calculation*/
			sensorUnits[i].calc_output(sensorUnits[0]);
		}

		/* output file */
		for (int i = DoF; i > 0; i--) {
			result << i << "," << sensorUnits[i].output << ","
				<< teach_NN[wave][i - 1] << endl;
		}
		result << "0,0,0" << endl << endl;	// base
	}


	fileClose();	//open output files

	return 0;
}
