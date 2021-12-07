/*----------------------------------------------*/
/*						*/
/*	define.h				*/
/*	Rybakプログラム用ヘッダファイル		*/
/*						*/
/*----------------------------------------------*/

#ifndef DEFINE_H_INCLUDED
#define DEFINE_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "../lib/hrp3pio.h"
#include "../lib/tsc-sh.h"
#include <linux/art_task.h>

// 計算用 --------------------------------------------------------------------------------
#define	PI		(3.14159265359)	// 円周率π
#define	pow2(x)		(x*x)		// 2乗
#define	pow4(x)		(x*x*x*x)	// 4乗
#define radtodeg  	(180.0/PI)	// [rad]→[deg]
#define degtorad  	(PI/180.0)	// [deg]→[rad]
#define l(L,Lopt) 	(L/Lopt)	// 標準化後筋肉長


// 要素数 --------------------------------------------------------------------------------
#define	UNITS		4   	// 4脚仕様設定
#define JOINTS_PER_LEG	3	// 一脚あたりの関節数
#define MUSCLES_PER_LEG	6	// 一脚あたりの筋肉の本数
#define SAMPLE		8400	// 最大サンプリングデータ数

#define	MAT_C_INPUTS	2	// 上位入力の数（d1, d2）
#define	MAT_A_INPUTS	12	// 興奮性結合するニューロン数
#define	MAT_B_INPUTS	10	// 抑制性結合するニューロン数
#define	MAT_W_INPUTS	5	// フィードバック系統（脚負荷以外）の数

#define	DEF_A	37	//Type1(V)+Type2(V)+Type1(hNaP)
#define	DEF_B	25	//Type1+Type2
#define	DEF_C	12	//Type1

// 脚番号 --------------------------------------------------------------------------------
#define LF	0	// 左前
#define LH	1	// 左後
#define RF	2	// 右前
#define RH	3	// 右後

// ニューロン識別番号テーブル ------------------------------------------------------------
//Type1-------------------
//RG
#define	RG_F	0
#define	RG_E	1
//PF
#define	PF_Sw	2
#define	PF_St	3
#define	PF_Lo	4
#define PF_Td	5
//Mn
#define	Mn_1	6
#define	Mn_2	7
#define	Mn_3	8
#define	Mn_4	9
#define	Mn_5	10
#define	Mn_6	11
//------------------------
//Type2-------------------
//In
#define	In_F		12
#define	In_E		13
#define	In      	14
#define	Inab_E		15
#define	In_PF_Sw	16
#define	In_PF_St	17
#define	In_PF_Lo	18
#define In_PF_Td	19
//Fb
#define	Fb_1		20
#define	Fb_2		21
#define	Fb_3		22
#define	Fb_4		23
//20191126追加　脚負荷が通る介在ニューロン
#define	In_Feed		24
//------------------------

#define	PF_NEURONS(x)	(x==PF_Sw || x==PF_St || x==PF_Lo || x==PF_Td)
#define	MN_NEURONS(x)	(x==Mn_1 || x==Mn_2 || x==Mn_3 || x==Mn_4 || x==Mn_5 || x==Mn_6)


// HRP関連 -------------------------------------------------------------------------------
#define num_ENC 16
#define	num_DIO	32
#define	num_PWM	16
#define num_AD	16
#define	PINS	24

#define	DIO	0
#define	PWM	1

#define	OFF	0
#define	ON	1
#define	KEEP	2

#define	P_DOWN	0
#define	P_UP	1


// 胴体傾斜関連 --------------------------------------------------------------------
#define num_AXIS	2
#define pitch		0
#define roll		1

/********************/
/*		    */
/*      構造体      */
/*		    */
/********************/
// ユニット定義 --------------------------------------------------------------------------
typedef struct rybakunit {
	float		Neuron_V[DEF_B];
	float		Neuron_hNaP[DEF_C];

	float		Muscle_Len[MUSCLES_PER_LEG];	// 各筋肉長さ[mm]

//	float		feed_prs;	// Foot pressure feedback
//	float		fb_trig;	// Hip trigger

	// CPG周期計算用パラメータ
	float		cpg_cycle;	// CPG周期
	int		cycle_flag;	// CPG周期計算用フラグ
	float		timing_CPG;	// RG-Fの発火タイミング（CPG周期計算用）
	float		RG_F_period;	// RG-F発火期間
	int		F_period_flag;	// RG-F発火期間計算用フラグ
	float		timing_RG_F;	// RG-Fの発火タイミング
	float		RG_E_period;	// RG-E発火期間
	int		E_period_flag;	// RG-E発火期間計算用フラグ
	float		timing_RG_E;	// RG-Eの発火タイミング

	int		touch;		// 接地情報(0:非接地、1:接地)
} RybakUnit;

// エンコーダデータ格納 ------------------------------------------------------------------
typedef struct{
	float angle_hip_deg[UNITS];		//腰関節角度[deg]
	float angle_knee_deg[UNITS];		//膝関節角度[deg]
	float angle_ankle_deg[UNITS];		//足首関節角度[deg]

	float angle_treadmill_rad;		//トレッドミル角度[rad]
	float speed_treadmill;			//トレッドミル速度
} ENCType;

// サンプリングデータ格納 ----------------------------------------------------------------
typedef struct{
	float	t[SAMPLE];	//経過時間
	float	cpg_cycle[UNITS][SAMPLE];	//CPG周期
	float	RG_F_period[UNITS][SAMPLE]; //RG-F発火期間
	float	RG_E_period[UNITS][SAMPLE]; //RG-E発火期間
	//RGニューロンの膜電位
	float	RG_F_V[UNITS][SAMPLE];
	float	RG_E_V[UNITS][SAMPLE];
	//PFニューロンの膜電位
	float	PF_Sw_V[UNITS][SAMPLE];
	float	PF_St_V[UNITS][SAMPLE];
	float	PF_Lo_V[UNITS][SAMPLE];
	float	PF_Td_V[UNITS][SAMPLE];
	//運動ニューロンの膜電位
	float	Mn_1_V[UNITS][SAMPLE];
	float	Mn_2_V[UNITS][SAMPLE];
	float	Mn_3_V[UNITS][SAMPLE];
	float	Mn_4_V[UNITS][SAMPLE];
	float	Mn_5_V[UNITS][SAMPLE];
	float	Mn_6_V[UNITS][SAMPLE];
	//脚負荷
	float	feed_prs[UNITS][SAMPLE];
	//関節角度
	float	angle_hip[UNITS][SAMPLE];
	float	angle_knee[UNITS][SAMPLE];
	float	angle_ankle[UNITS][SAMPLE];
	//FF電圧値
	float	FF_data[UNITS][SAMPLE];
	//接地情報(FF依存)
	int	FF_touch[UNITS][SAMPLE];
	//ポテンショメータ電圧値
	float	POT_data[UNITS][SAMPLE];
	//足底プレート角度
	float	foot_angle[UNITS][SAMPLE];
	//足底プレート角速度
	float	foot_angle_vel[UNITS][SAMPLE];
	//接地情報(ポテンショメータ依存)
	int	POT_touch[UNITS][SAMPLE];
	//接地情報(決定版)
	int	touch[UNITS][SAMPLE];
	//ジャイロセンサ出力
	float gyro_data[num_AXIS][SAMPLE];
	//胴体傾き角度(ジャイロセンサ)
	float gyro_angle_deg[num_AXIS][SAMPLE];
	//ローパス通した後の胴体傾き角度(ジャイロセンサ)
	float gyro_angle_lowp_deg[num_AXIS][SAMPLE];
	//加速度センサ出力
	float acc_data[num_AXIS][SAMPLE];
	//胴体傾き角度(加速度センサ)
	float acc_angle_deg[num_AXIS][SAMPLE];
	//ローパス通した後の胴体傾き角度(加速度センサ)
	float acc_angle_lowp_deg[num_AXIS][SAMPLE];
	//胴体傾き角度(決定版)
	float tilt_angle_deg[num_AXIS][SAMPLE];
	//トレッドミル速度
	float speed_treadmill[SAMPLE];
	//人工筋肉内気圧
	float atmosphere[SAMPLE];
	//20191126追加　脚負荷用介在ニューロンIn_Feed
	float	In_Feed_V[UNITS][SAMPLE];
    //筋肉の動作状況
    unsigned int muscle_io[SAMPLE];
    //筋肉動作モードの保存
    int Valve_mode1[UNITS][SAMPLE];
    int Valve_mode2[UNITS][SAMPLE];
    int Valve_mode3[UNITS][SAMPLE];
    int Valve_mode4[UNITS][SAMPLE];
    int Valve_mode5[UNITS][SAMPLE];
    int Valve_mode6[UNITS][SAMPLE];
} SAMType;

// 脚負荷データ格納 ----------------------------------------------------------------------
typedef struct {
	int	FF_touch[UNITS];		// FlexiForce依存の接地情報（0:非接地，1～4:接地）
	int	POT_touch[UNITS];		// ポテンショメータ依存の接地情報（0:非接地，1～4:接地）
	int	touch[UNITS];			// 接地情報（0:非接地，1～4:接地）
	float	load_char[UNITS];		// 設定空気圧における収縮率に対する張力特性
	float	load_value[UNITS];		// 脚負荷[N](エンコーダからの推定値
} LOADType;

// ADポート出力データ格納 ----------------------------------------------------------------
typedef struct{
	// FlexiForce関連
	float FF_data[UNITS];				// FlexiForce出力[V]
	float FF_data_offset[UNITS];			// FlexiForceオフセット出力[V]

	// ポテンショメータ関連
	float POT_data[UNITS];				// ポテンショメータ出力[V]
	float POT_data_offset[UNITS];			// ポテンショメータオフセット出力[V]
	float foot_angle[UNITS];			// 足底プレートの角度[deg]
	float foot_angle_offset[UNITS];			// オフセット角度[deg]
	float foot_rel_angle[UNITS];			// オフセット角度を基準とした相対角度[deg]
	float foot_angle_last[UNITS];			// ひとつ前の足底プレートの角度[deg]
	float foot_angle_vel[UNITS];			// 足底プレートの角速度[deg/sec]

	// ジャイロセンサ関連
	float gyro_data[num_AXIS];			// ジャイロセンサ出力[V]
	float gyro_data_offset[num_AXIS];		// ジャイロセンサオフセット出力[V]
	float gyro_angle_vel[num_AXIS];			// 角速度[deg/sec]
	float gyro_angle_deg[num_AXIS];			// 角度[deg]
	float gyro_angle_rad[num_AXIS];			// 角度[rad]
	float gyro_angle_lowp_deg[num_AXIS];		// ローパス通した後の角度[deg]
	float gyro_angle_lowp_rad[num_AXIS];		// ローパス通した後の角度[rad]
	float gyro_angle_lowp_rad_last[num_AXIS];	// ひとつ前のローパス通した後の角度[rad]

	// 加速度センサ関連
	float acc_data[num_AXIS];			// 加速度センサ出力[V]
	float acc_data_offset[num_AXIS];		// 加速度センサオフセット出力[V]
	float acc_angle_deg[num_AXIS];			// 角度[deg]
	float acc_angle_rad[num_AXIS];			// 角度[rad]
	float acc_angle_lowp_deg[num_AXIS];		// ローパス通した後の角度[deg]
	float acc_angle_lowp_rad[num_AXIS];		// ローパス通した後の角度[rad]
	float acc_angle_lowp_rad_last[num_AXIS];	// ひとつ前のローパス通した後の角度[rad]

	// 胴体傾斜関連
	float tilt_angle_deg[num_AXIS];			// 胴体傾斜[deg]
	float tilt_angle_rad[num_AXIS];			// 胴体傾斜[rad]
	float tilt_angle_offset_deg[num_AXIS];		// 胴体傾斜オフセット[deg]
	float tilt_angle_offset_rad[num_AXIS];		// 胴体傾斜オフセット[rad]

	// 空気圧センサ関連
	float air_press_data;				// 空気圧センサ出力[V]
	float atmosphere;				// 空気圧[MPa]
} ADType;

// 筋肉の動作状況データ格納
unsigned int muscle_io;
int valve_mode[UNITS][MUSCLES_PER_LEG];     //電磁弁の開放率のモード


//------関数のプロトタイプ宣言------//
// mSynCPG_robot.c
//float CalcLength(int i, RybakUnit *Unit);
void InitUnit(RybakUnit Unit[UNITS]);
float GetfV(float V, int x);
void aux(int x, float y[DEF_A], float f[DEF_A], float iF, float iE, float eF, float eE, float d[2], float h, float MATc[][DEF_B], float MATa[][DEF_B], float MATb[][DEF_B], float MATw[][DEF_B], ENCType *enc, LOADType *load);
void CalcUnit_RK2(RybakUnit Unit[UNITS], float d[2], float h, ENCType *enc, LOADType *load);
void CalcUnit_RK4(RybakUnit Unit[UNITS], float d[2], float h, ENCType *enc, LOADType *load);
void CalcCycle(RybakUnit Unit[UNITS], float t);

// sensor.c
//void angle_get(int i, ENCType *enc);
void all_angle_get(ENCType *enc);
void speed_treadmill_get(ENCType *enc, float h);
//void load_get(int i, LOADType *load, RybakUnit *Unit);
void measure_offset_sum(ADType *ad);
void measure_offset_con(int k, ADType *ad);
void all_FF_get(ADType *ad, double ad_data[]);
void all_POT_get(ADType *ad, double ad_data[], float h);
void touch_get(ADType *ad, LOADType *load);
void gyro_get(ADType *ad, double ad_data[], float h);
void acc_get(ADType *ad, double ad_data[], float h);
void tilt_con(ADType *ad);
void air_press_get(ADType *ad, double ad_data[]);



// muscleController.c
void set_valve(int x, int ONOFF);
void setValves(int i, int muscle, int state);
float Mn_Addition(int i, int j, RybakUnit Unit[UNITS]);
int Mn_Average(int i,int j);
void control_Muscles(float t, RybakUnit Unit[UNITS]);
void muscle_flexion();
void muscle_extension();
void muscle_reset();
void muscle_onoff(int unit, int muscle, int ONOFF);

// function.c
void data_get(int k, float t, SAMType *sam, RybakUnit Unit[], ENCType *enc, ADType *ad, LOADType *load);
int file_input(float buf[30][30], float c_f[][DEF_B], float a_f[][DEF_B], float b_f[][DEF_B], float w_f[][DEF_B], float c_h[][DEF_B], float a_h[][DEF_B], float b_h[][DEF_B], float w_h[][DEF_B]);
int file_output(int k, SAMType *sam);
void param(float d[2], LOADType *load, float t);
void Calc_proc_time(int k, float t_p_sum, float t_p_max);

// math_fast.c
void Muscle_len_table_init();
float Calc_Muscle6_F_len(float angle_shoulder, float angle_elbow);
float Calc_Muscle6_H_len(float angle_knee, float angle_ankle);
void Aux_table_init();
float Calc_mK(int i);
float Calc_mNaP(int i);
float Calc_hInfNaP(int i);
float Calc_tauhNaP_RG_inv(int i);
float Calc_tauhNaP_inv(int i);
void fV_table_init();
float Calc_fV_Mn(int i);
float Calc_fV(int i);

// udp_transmission.c
void udp_transmission(float t);

#endif
