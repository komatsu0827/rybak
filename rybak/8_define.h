/*----------------------------------------------*/
/*						*/
/*	define.h				*/
/*	Rybak�v���O�����p�w�b�_�t�@�C��		*/
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

// �v�Z�p --------------------------------------------------------------------------------
#define	PI		(3.14159265359)	// �~������
#define	pow2(x)		(x*x)		// 2��
#define	pow4(x)		(x*x*x*x)	// 4��
#define radtodeg  	(180.0/PI)	// [rad]��[deg]
#define degtorad  	(PI/180.0)	// [deg]��[rad]
#define l(L,Lopt) 	(L/Lopt)	// �W������ؓ���


// �v�f�� --------------------------------------------------------------------------------
#define	UNITS		4   	// 4�r�d�l�ݒ�
#define JOINTS_PER_LEG	3	// ��r������̊֐ߐ�
#define MUSCLES_PER_LEG	6	// ��r������̋ؓ��̖{��
#define SAMPLE		8400	// �ő�T���v�����O�f�[�^��

#define	MAT_C_INPUTS	2	// ��ʓ��͂̐��id1, d2�j
#define	MAT_A_INPUTS	12	// ��������������j���[������
#define	MAT_B_INPUTS	10	// �}������������j���[������
#define	MAT_W_INPUTS	5	// �t�B�[�h�o�b�N�n���i�r���׈ȊO�j�̐�

#define	DEF_A	37	//Type1(V)+Type2(V)+Type1(hNaP)
#define	DEF_B	25	//Type1+Type2
#define	DEF_C	12	//Type1

// �r�ԍ� --------------------------------------------------------------------------------
#define LF	0	// ���O
#define LH	1	// ����
#define RF	2	// �E�O
#define RH	3	// �E��

// �j���[�������ʔԍ��e�[�u�� ------------------------------------------------------------
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
//20191126�ǉ��@�r���ׂ��ʂ��݃j���[����
#define	In_Feed		24
//------------------------

#define	PF_NEURONS(x)	(x==PF_Sw || x==PF_St || x==PF_Lo || x==PF_Td)
#define	MN_NEURONS(x)	(x==Mn_1 || x==Mn_2 || x==Mn_3 || x==Mn_4 || x==Mn_5 || x==Mn_6)


// HRP�֘A -------------------------------------------------------------------------------
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


// ���̌X�Ί֘A --------------------------------------------------------------------
#define num_AXIS	2
#define pitch		0
#define roll		1

/********************/
/*		    */
/*      �\����      */
/*		    */
/********************/
// ���j�b�g��` --------------------------------------------------------------------------
typedef struct rybakunit {
	float		Neuron_V[DEF_B];
	float		Neuron_hNaP[DEF_C];

	float		Muscle_Len[MUSCLES_PER_LEG];	// �e�ؓ�����[mm]

//	float		feed_prs;	// Foot pressure feedback
//	float		fb_trig;	// Hip trigger

	// CPG�����v�Z�p�p�����[�^
	float		cpg_cycle;	// CPG����
	int		cycle_flag;	// CPG�����v�Z�p�t���O
	float		timing_CPG;	// RG-F�̔��΃^�C�~���O�iCPG�����v�Z�p�j
	float		RG_F_period;	// RG-F���Ί���
	int		F_period_flag;	// RG-F���Ί��Ԍv�Z�p�t���O
	float		timing_RG_F;	// RG-F�̔��΃^�C�~���O
	float		RG_E_period;	// RG-E���Ί���
	int		E_period_flag;	// RG-E���Ί��Ԍv�Z�p�t���O
	float		timing_RG_E;	// RG-E�̔��΃^�C�~���O

	int		touch;		// �ڒn���(0:��ڒn�A1:�ڒn)
} RybakUnit;

// �G���R�[�_�f�[�^�i�[ ------------------------------------------------------------------
typedef struct{
	float angle_hip_deg[UNITS];		//���֐ߊp�x[deg]
	float angle_knee_deg[UNITS];		//�G�֐ߊp�x[deg]
	float angle_ankle_deg[UNITS];		//����֐ߊp�x[deg]

	float angle_treadmill_rad;		//�g���b�h�~���p�x[rad]
	float speed_treadmill;			//�g���b�h�~�����x
} ENCType;

// �T���v�����O�f�[�^�i�[ ----------------------------------------------------------------
typedef struct{
	float	t[SAMPLE];	//�o�ߎ���
	float	cpg_cycle[UNITS][SAMPLE];	//CPG����
	float	RG_F_period[UNITS][SAMPLE]; //RG-F���Ί���
	float	RG_E_period[UNITS][SAMPLE]; //RG-E���Ί���
	//RG�j���[�����̖��d��
	float	RG_F_V[UNITS][SAMPLE];
	float	RG_E_V[UNITS][SAMPLE];
	//PF�j���[�����̖��d��
	float	PF_Sw_V[UNITS][SAMPLE];
	float	PF_St_V[UNITS][SAMPLE];
	float	PF_Lo_V[UNITS][SAMPLE];
	float	PF_Td_V[UNITS][SAMPLE];
	//�^���j���[�����̖��d��
	float	Mn_1_V[UNITS][SAMPLE];
	float	Mn_2_V[UNITS][SAMPLE];
	float	Mn_3_V[UNITS][SAMPLE];
	float	Mn_4_V[UNITS][SAMPLE];
	float	Mn_5_V[UNITS][SAMPLE];
	float	Mn_6_V[UNITS][SAMPLE];
	//�r����
	float	feed_prs[UNITS][SAMPLE];
	//�֐ߊp�x
	float	angle_hip[UNITS][SAMPLE];
	float	angle_knee[UNITS][SAMPLE];
	float	angle_ankle[UNITS][SAMPLE];
	//FF�d���l
	float	FF_data[UNITS][SAMPLE];
	//�ڒn���(FF�ˑ�)
	int	FF_touch[UNITS][SAMPLE];
	//�|�e���V�����[�^�d���l
	float	POT_data[UNITS][SAMPLE];
	//����v���[�g�p�x
	float	foot_angle[UNITS][SAMPLE];
	//����v���[�g�p���x
	float	foot_angle_vel[UNITS][SAMPLE];
	//�ڒn���(�|�e���V�����[�^�ˑ�)
	int	POT_touch[UNITS][SAMPLE];
	//�ڒn���(�����)
	int	touch[UNITS][SAMPLE];
	//�W���C���Z���T�o��
	float gyro_data[num_AXIS][SAMPLE];
	//���̌X���p�x(�W���C���Z���T)
	float gyro_angle_deg[num_AXIS][SAMPLE];
	//���[�p�X�ʂ�����̓��̌X���p�x(�W���C���Z���T)
	float gyro_angle_lowp_deg[num_AXIS][SAMPLE];
	//�����x�Z���T�o��
	float acc_data[num_AXIS][SAMPLE];
	//���̌X���p�x(�����x�Z���T)
	float acc_angle_deg[num_AXIS][SAMPLE];
	//���[�p�X�ʂ�����̓��̌X���p�x(�����x�Z���T)
	float acc_angle_lowp_deg[num_AXIS][SAMPLE];
	//���̌X���p�x(�����)
	float tilt_angle_deg[num_AXIS][SAMPLE];
	//�g���b�h�~�����x
	float speed_treadmill[SAMPLE];
	//�l�H�ؓ����C��
	float atmosphere[SAMPLE];
	//20191126�ǉ��@�r���חp��݃j���[����In_Feed
	float	In_Feed_V[UNITS][SAMPLE];
    //�ؓ��̓����
    unsigned int muscle_io[SAMPLE];
    //�ؓ����샂�[�h�̕ۑ�
    int Valve_mode1[UNITS][SAMPLE];
    int Valve_mode2[UNITS][SAMPLE];
    int Valve_mode3[UNITS][SAMPLE];
    int Valve_mode4[UNITS][SAMPLE];
    int Valve_mode5[UNITS][SAMPLE];
    int Valve_mode6[UNITS][SAMPLE];
} SAMType;

// �r���׃f�[�^�i�[ ----------------------------------------------------------------------
typedef struct {
	int	FF_touch[UNITS];		// FlexiForce�ˑ��̐ڒn���i0:��ڒn�C1�`4:�ڒn�j
	int	POT_touch[UNITS];		// �|�e���V�����[�^�ˑ��̐ڒn���i0:��ڒn�C1�`4:�ڒn�j
	int	touch[UNITS];			// �ڒn���i0:��ڒn�C1�`4:�ڒn�j
	float	load_char[UNITS];		// �ݒ��C���ɂ�������k���ɑ΂��钣�͓���
	float	load_value[UNITS];		// �r����[N](�G���R�[�_����̐���l
} LOADType;

// AD�|�[�g�o�̓f�[�^�i�[ ----------------------------------------------------------------
typedef struct{
	// FlexiForce�֘A
	float FF_data[UNITS];				// FlexiForce�o��[V]
	float FF_data_offset[UNITS];			// FlexiForce�I�t�Z�b�g�o��[V]

	// �|�e���V�����[�^�֘A
	float POT_data[UNITS];				// �|�e���V�����[�^�o��[V]
	float POT_data_offset[UNITS];			// �|�e���V�����[�^�I�t�Z�b�g�o��[V]
	float foot_angle[UNITS];			// ����v���[�g�̊p�x[deg]
	float foot_angle_offset[UNITS];			// �I�t�Z�b�g�p�x[deg]
	float foot_rel_angle[UNITS];			// �I�t�Z�b�g�p�x����Ƃ������Ίp�x[deg]
	float foot_angle_last[UNITS];			// �ЂƂO�̑���v���[�g�̊p�x[deg]
	float foot_angle_vel[UNITS];			// ����v���[�g�̊p���x[deg/sec]

	// �W���C���Z���T�֘A
	float gyro_data[num_AXIS];			// �W���C���Z���T�o��[V]
	float gyro_data_offset[num_AXIS];		// �W���C���Z���T�I�t�Z�b�g�o��[V]
	float gyro_angle_vel[num_AXIS];			// �p���x[deg/sec]
	float gyro_angle_deg[num_AXIS];			// �p�x[deg]
	float gyro_angle_rad[num_AXIS];			// �p�x[rad]
	float gyro_angle_lowp_deg[num_AXIS];		// ���[�p�X�ʂ�����̊p�x[deg]
	float gyro_angle_lowp_rad[num_AXIS];		// ���[�p�X�ʂ�����̊p�x[rad]
	float gyro_angle_lowp_rad_last[num_AXIS];	// �ЂƂO�̃��[�p�X�ʂ�����̊p�x[rad]

	// �����x�Z���T�֘A
	float acc_data[num_AXIS];			// �����x�Z���T�o��[V]
	float acc_data_offset[num_AXIS];		// �����x�Z���T�I�t�Z�b�g�o��[V]
	float acc_angle_deg[num_AXIS];			// �p�x[deg]
	float acc_angle_rad[num_AXIS];			// �p�x[rad]
	float acc_angle_lowp_deg[num_AXIS];		// ���[�p�X�ʂ�����̊p�x[deg]
	float acc_angle_lowp_rad[num_AXIS];		// ���[�p�X�ʂ�����̊p�x[rad]
	float acc_angle_lowp_rad_last[num_AXIS];	// �ЂƂO�̃��[�p�X�ʂ�����̊p�x[rad]

	// ���̌X�Ί֘A
	float tilt_angle_deg[num_AXIS];			// ���̌X��[deg]
	float tilt_angle_rad[num_AXIS];			// ���̌X��[rad]
	float tilt_angle_offset_deg[num_AXIS];		// ���̌X�΃I�t�Z�b�g[deg]
	float tilt_angle_offset_rad[num_AXIS];		// ���̌X�΃I�t�Z�b�g[rad]

	// ��C���Z���T�֘A
	float air_press_data;				// ��C���Z���T�o��[V]
	float atmosphere;				// ��C��[MPa]
} ADType;

// �ؓ��̓���󋵃f�[�^�i�[
unsigned int muscle_io;
int valve_mode[UNITS][MUSCLES_PER_LEG];     //�d���ق̊J�����̃��[�h


//------�֐��̃v���g�^�C�v�錾------//
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
