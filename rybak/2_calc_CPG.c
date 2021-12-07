// ---------------------------- //
//	CPG�v�Z�p�v���O����	//
//	calc_CPG.c		//
//	(calculation_CPG)	//
// ---------------------------- //

#include "8_define.h"
#include "9_user_param.h"

// Ia�CIb�CII�Ɋւ��萔
//#define	vnorm(v)	(v/59.0)// norm�͕W������iLth = 59mm�j
#define vnorm(v, Lth)	(v/Lth)	// norm�͕W������ (Lth = 59mm)
#define	pv		0.6
#define	kv		0.5//6.2
#define	knI		0.0//0.06
#define	kF		1.0
#define	kdII		1.5
#define	knII		0.06
//#define	Lth		59.0	// ���S����������z�N������ŏ��ؓ����i臒l�j
//#define	Fth		3.38

// V�Ɋւ���萔
const float	_gK =		4.5e-6;		// [nS] �ő�R���_�N�^���X
const float	_gLeak =	1.6e-6;
const float	_gSynE =	10.0e-6;
const float	_gSynI =	10.0e-6;
const float	_gNaP_RG =	3.5e-6;
const float	_gNaP_PF =	0.5e-6;
const float	_gNaP_Mn =	0.3e-6;
const float	C =		20.0e-9;	// [pF] ��������v��
#define		ENa             55.0		// [mV] �Ή�����t�]�d��
#define		EK              (-80.0)
#define		ESynE           (-10.0)
#define		ESynI           (-70.0)
#define		ELeak           (-64.0)
#define		ELeakIn         (-60.0)
#define		tauhNaPmax_RG   300.0e-3	//300.0e-3; // [mS]��[��S]? ��������v���F600.0��1000����1��?
#define		tauhNaPmax      600.0
#define		Vth             (-50.0)


// �O�r�p�����׏d�̊i�[�ꏊ
float MATc_fore[MAT_C_INPUTS][DEF_B];
float MATa_fore[MAT_A_INPUTS][DEF_B];
float MATb_fore[MAT_B_INPUTS][DEF_B];
float MATw_fore[MAT_W_INPUTS][DEF_B];
// ��r�p�����׏d�̊i�[�ꏊ
float MATc_hind[MAT_C_INPUTS][DEF_B];
float MATa_hind[MAT_A_INPUTS][DEF_B];
float MATb_hind[MAT_B_INPUTS][DEF_B];
float MATw_hind[MAT_W_INPUTS][DEF_B];


// �r�Ԍ���(�}����)							//LF		LH		RF		RH
float	c_F[UNITS][UNITS] = {	/*LF*/	{ 0.0,		0.0,		U2U_LR, 	0.0 	},
                                /*LH*/	{ 0.0,		0.0, 		0.0, 		U2U_LR 	},
                                /*RF*/	{ U2U_LR, 	0.0, 		0.0, 		0.0 	},
                                /*RH*/	{ 0.0, 		U2U_LR, 	0.0, 		0.0 	}	};

											//LF		LH		RF		RH
float	c_E[UNITS][UNITS] = {	/*LF*/	{ 0.0,		0.0,		U2U_LR, 	0.0 	},
								/*LH*/	{ 0.0,		0.0, 		0.0, 		U2U_LR 	},
								/*RF*/	{ U2U_LR, 	0.0, 		0.0, 		0.0 	},
								/*RH*/	{ 0.0, 		U2U_LR, 	0.0, 		0.0 	} };

// �r�Ԍ���(������)							//LF		LH		RF		RH
float	b_F[UNITS][UNITS] = {	/*LF*/	{ 0.0,		U1U_FH,		0.0, 		0.0 	},
								/*LH*/	{ U1U_FH,	0.0, 		0.0, 		0.0 	},
								/*RF*/	{ 0.0,	 	0.0, 		0.0, 		U1U_FH 	},
								/*RH*/	{ 0.0, 		0.0,	 	U1U_FH, 	0.0 	} };

											//LF		LH		RF		RH
float	b_E[UNITS][UNITS] = {	/*LF*/	{ 0.0,		U1U_FH,		0.0, 		0.0 	},
								/*LH*/	{ U1U_FH,	0.0, 		0.0, 		0.0 	},
								/*RF*/	{ 0.0,	 	0.0, 		0.0, 		U1U_FH 	},
								/*RH*/	{ 0.0, 		0.0,	 	U1U_FH, 	0.0 	} };



/**************************/
/*                        */
/*        �����֐�        */
/*                        */
/**************************/
/*//�ؓ��̒����v�Z
float CalcLength(int i, RybakUnit *Unit)
{
	double	ret = 0.0;	//�ؒ�
	double	bx1, by1, bx2, by2;	//�ؒ��v�Z�p�p�����[�^

	// 1�ԋؒ����i��/���b�����؁j
	if (i==0) {
		bx1 = cos(-Unit->tht[0]) * (Unit->Llen[0] + Unit->Muscle[i].bx) - sin(-Unit->tht[0]) * Unit->Muscle[i].by;
		by1 = sin(-Unit->tht[0]) * (Unit->Llen[0] + Unit->Muscle[i].bx) + cos(-Unit->tht[0]) * Unit->Muscle[i].by;
		ret = sqrt(pow2(bx1 - Unit->Muscle[i].ax) + pow2(by1 - Unit->Muscle[i].ay));
	}
	// 2�ԋؒ����i��/���b���L�؁j
	else if (i==1) {
		bx1 = cos(-Unit->tht[0]) * (Unit->Llen[0] + Unit->Muscle[i].bx) - sin(-Unit->tht[0]) * Unit->Muscle[i].by;
		by1 = sin(-Unit->tht[0]) * (Unit->Llen[0] + Unit->Muscle[i].bx) + cos(-Unit->tht[0]) * Unit->Muscle[i].by;
		ret = sqrt(pow2(bx1 - Unit->Muscle[i].ax) + pow2(by1 - Unit->Muscle[i].ay));
	}
	// 3�ԋؒ����i�G/�����؁j
	else if (i==2) {
		bx1 = cos(-Unit->tht[1]) * Unit->Muscle[i].bx - sin(-Unit->tht[1]) * Unit->Muscle[i].by + Unit->Llen[0];
		by1 = sin(-Unit->tht[1]) * Unit->Muscle[i].bx + cos(-Unit->tht[1]) * Unit->Muscle[i].by;
		bx2 = cos(-Unit->tht[0]) * bx1 - sin(-Unit->tht[0]) * by1;
		by2 = sin(-Unit->tht[0]) * bx1 + cos(-Unit->tht[0]) * by1;
		ret = sqrt(pow2(bx2 - Unit->Muscle[i].ax) + pow2(by2 - Unit->Muscle[i].ay));
	}
//	// 4�ԋؒ����i�G/���L�؁j
	else if (i==3) {
		bx1 = cos(-Unit->tht[1]) * Unit->Muscle[i].bx - sin(-Unit->tht[1]) * Unit->Muscle[i].by + Unit->Llen[0];
		by1 = sin(-Unit->tht[1]) * Unit->Muscle[i].bx + cos(-Unit->tht[1]) * Unit->Muscle[i].by;
		ret = sqrt(pow2(bx1 - Unit->Muscle[i].ax) + pow2(by1 - Unit->Muscle[i].ay));
	}
//	// 5�ԋؒ����i����/�I���؁j
	else if (i==4) {
		bx1 = cos(Unit->tht[2]) * Unit->Muscle[i].bx - sin(Unit->tht[2]) * Unit->Muscle[i].by + Unit->Llen[1];
		by1 = sin(Unit->tht[2]) * Unit->Muscle[i].bx + cos(Unit->tht[2]) * Unit->Muscle[i].by;
		ret = sqrt(pow2(bx1 - Unit->Muscle[i].ax) + pow2(by1 - Unit->Muscle[i].ay));
	}
	// 6�ԋؒ����i����/�I�L�؁j
	else if (i==5) {
		bx1 = cos(Unit->tht[2]) * Unit->Muscle[i].bx - sin(Unit->tht[2]) * Unit->Muscle[i].by + Unit->Llen[1];
		by1 = sin(Unit->tht[2]) * Unit->Muscle[i].bx + cos(Unit->tht[2]) * Unit->Muscle[i].by;
		bx2 = cos(-Unit->tht[1]) * bx1 - sin(-Unit->tht[1]) * by1;
		by2 = sin(-Unit->tht[1]) * bx1 + cos(-Unit->tht[1]) * by1;
		ret = sqrt(pow2(bx2 - Unit->Muscle[i].ax) + pow2(by2 - Unit->Muscle[i].ay));
	}

	return (float)ret;
}
*/

/**************************/
/*                        */
/*       ��p�֐��Q       */
/*                        */
/**************************/
void InitUnit(RybakUnit Unit[UNITS])
{
	int	i, j;
	float	buf[30][30];

	// �j���[���������l
	float	Vdef[UNITS][DEF_B] = {
				//RG_F	RG_E		PF_Sw		PF_St		PF_Lo		PF_Td		Mn_1		Mn_2		Mn_3		Mn_4		Mn_5		Mn_6		In_F		In_E		In			Inab_E	In_PF_Sw	In_PF_St	In_PF_Lo	In_PF_Td	Fb_1		Fb_2		Fb_3		Fb_4		In_Feed(20191126�ǉ�)
			{	-20.0,	-40.0,	-20.0,	-40.0,	-30.0,	-30.0,	-40.0,	-40.0,	-40.0,	-40.0,	-40.0,	-40.0,	-20.0,	-40.0,	-20.0,	-40.0,	-40.0,		-40.0,		-40.0,		-40.0,		-40.0,	-40.0,	-40.0,	-40.0,	-40.0	},	// ���O(�)
			{	-40.0,	-20.0,	-40.0,	-20.0,	-30.0,	-30.0,	-40.0,	-40.0,	-40.0,	-40.0,	-40.0,	-40.0,	-40.0,	-20.0,	-40.0,	-20.0,	-40.0,		-40.0,		-40.0,		-40.0,		-40.0,	-40.0,	-40.0,	-40.0,	-40.0	},	// ����(���@�@�Ƌt�ʑ��̂���Ext����Flx���ŋt�̒l)
			{	-40.0,	-20.0,	-40.0,	-20.0,	-30.0,	-30.0,	-40.0,	-40.0,	-40.0,	-40.0,	-40.0,	-40.0,	-40.0,	-20.0,	-40.0,	-20.0,	-40.0,		-40.0,		-40.0,		-40.0,		-40.0,	-40.0,	-40.0,	-40.0,	-40.0	},	// �E�O(�����@�Ƌt�ʑ��̂���Ext����Flx���ŋt�̒l)
			{	-20.0,	-40.0,	-20.0,	-40.0,	-30.0,	-30.0,	-40.0,	-40.0,	-40.0,	-40.0,	-40.0,	-40.0,	-20.0,	-40.0,	-20.0,	-40.0,	-40.0,		-40.0,		-40.0,		-40.0,		-40.0,	-40.0,	-40.0,	-40.0,	-40.0	}};	// �E��(�������Ɠ��ʑ��̂���Ext����Flx���œ����l)

							//RG_F	RG_E	PF_Sw	PF_St	PF_Lo	PF_Td	Mn_1	Mn_2	Mn_3	Mn_4	Mn_5	Mn_6
	float	hNaPdef[UNITS][DEF_C] = {	{	0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0,	0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0 },
						{	0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0,	0.0, 	0.0, 	0.0, 	0.0, 	0.0 },
						{	0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0,	0.0, 	0.0, 	0.0, 	0.0 },
						{	0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0,	0.0, 	0.0, 	0.0 }	};

	// �ǂݍ��݃t�@�C������S�����׏d���i�[
	file_input(buf, MATc_fore, MATa_fore, MATb_fore, MATw_fore, MATc_hind, MATa_hind, MATb_hind, MATw_hind);

	// �j���[����������
	for(i=0; i<UNITS; i++){
		// type1
		for (j=0; j<DEF_C; j++) {
			Unit[i].Neuron_V[j] = Vdef[i][j];
			Unit[i].Neuron_hNaP[j] = hNaPdef[i][j];
		}
		// type2
		for (j=DEF_C; j<DEF_B; j++) {
			Unit[i].Neuron_V[j] = Vdef[i][j];
		}
	}

	// �t�B�[�h�o�b�N�l�̏�����
//	for(i=0; i<UNITS; i++){
//		Unit[i].feed_prs = 0.0;
//		Unit[i].fb_trig = 0.0;
//		for(j=0; j<4; j++)
//			Unit[i].fb[j] = 0.0;
//	}
}

// �j���[�����̓d��V����f(V)���擾
float GetfV(float V, int x)	// V:�e�j���[�����̓d�ʁCx:�j���[�������ʔԍ�
{
	// ���S���u
	if (x<0 || DEF_B-1<x) {
		printf("\nError: �s���Ȓl���Q�Ƃ��悤�Ƃ��܂���.\n");
		exit(1);
	}

	// �j���[�����d��V���甭�Η�f(V)���Z�o
	if (V >= Vth) {
		if (MN_NEURONS(x))	// Mn
			return ( 1.0 / ( 1.0 + exp( -(V + 40.0) * 0.5 ) ) );
		else 	// RG, PF, In, Fb
			return ( 1.0 / ( 1.0 + exp( -(V + 30.0) * 0.125 ) ) );
	}
	else
		return 0.0;
}

/**************************/
/*                        */
/*      �����Q�N�b�^      */
/*                        */
/**************************/
void aux(int x, float y[DEF_A], float f[DEF_A], float iF, float iE, float eF, float eE, float d[2], float h, float MATc[][DEF_B], float MATa[][DEF_B], float MATb[][DEF_B], float MATw[][DEF_B], ENCType *enc, LOADType *load)
{
	int 	i;
	int 	i_V[DEF_B];
	float	fV[DEF_B];
//	float	Muscle_v[MUSCLES_PER_LEG];
//	float	dnorm;	// Ia-F, Ia-E, II-F �̌v�Z�Ɏg�p
//	float	Fnorm;	// Ib-E �̌v�Z�Ɏg�p
	float	Muscle_Len;
	float	fb[4] = {0.0};
	float	mK[DEF_C], mNaP[DEF_C], hInfNaP[DEF_C], INaP[DEF_C], tauhNaP_inv[DEF_C], IK[DEF_C];
	float	ILeak[DEF_B], ISynE[DEF_B], ISynI[DEF_B];
	float	feed_prs = 0.0;
	float	fb_trig = 0.0;


	// f(V)���v�Z
	for (i = 0; i<DEF_B; i++) {
//		i_V[i] = (int)( (y[i] - V_MIN) / (V_MAX - V_MIN) * (V_num - 1) );
		i_V[i] = (int)( (y[i] + 80.0) * 9.090909 + 0.00001);	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001�̎�

//		fV[i] = GetfV(y[i], i);
		if (MN_NEURONS(i))	// Mn
			fV[i] = Calc_fV_Mn(i_V[i]);
		else 	// RG, PF, In, Fb
			fV[i] = Calc_fV(i_V[i]);
	}

/*	// �ؓ�����̃t�B�[�h�o�b�Nfb[0]�`fb[3]�̌v�Z ------------------------------------------------------------------------------------------------
	// �p�����[�^����
	for (i=0; i<MUSCLES_PER_LEG; i++) //�������Ōv�Z����K�v����
		Muscle_v[i] = (l(Unit->Muscle[i].dL-Unit->Muscle[i].dLp, Unit->Muscle[i].Lopt)) / h;

	//------ Ia-F ------//
	if (Unit->Muscle[0].dL >= Unit->Lth[0])
		dnorm = (Unit->Muscle[0].dL - Unit->Lth[0]) / Unit->Lth[0];
	else
		dnorm = 0.0;
	if (0.0 <= Unit->Muscle[0].v)
		Unit->fb[0] = kv * pow(vnorm(Unit->Muscle[0].v, Unit->Lth[0]), pv) + Unit->kdI_hipF * dnorm + knI * fV[Mn_1] + Unit->constI;
	else
		Unit->fb[0] = -(kv * pow(-vnorm(Unit->Muscle[0].v, Unit->Lth[0]), pv)) + Unit->kdI_hipF * dnorm + knI * fV[Mn_1] + Unit->constI;

	//------ II-F ------//
	if (Unit->Muscle[0].dL >= Unit->Lth[0])
		dnorm = (Unit->Muscle[0].dL - Unit->Lth[0]) / Unit->Lth[0];
	else
		dnorm = 0.0;
	Unit->fb[1] = kdII * dnorm + Unit->kdI_hipF * dnorm + knII * fV[Mn_1] + Unit->constII;

	//------ Ia-E ------//
	if (Unit->Muscle[1].dL >= Unit->Lth[1])
		dnorm = (Unit->Muscle[1].dL - Unit->Lth[1]) / Unit->Lth[1];
	else
		dnorm = 0.0;
	if (0.0 <= Unit->Muscle[1].v)
		Unit->fb[2] = kv * pow(vnorm(Unit->Muscle[1].v, Unit->Lth[1]), pv) + Unit->kdI_hipE * dnorm + knI * fV[Mn_2] + Unit->constI;
	else
		Unit->fb[2] = -(kv * pow(-vnorm(Unit->Muscle[1].v, Unit->Lth[1]), pv)) + Unit->kdI_hipE * dnorm + knI * fV[Mn_2] + Unit->constI;

	//------ Ib-E ------//
	if (Unit->Muscle[1].F >= Unit->Fth)
		Fnorm = ((Unit->Muscle[1].F) - Unit->Fth)/Unit->Muscle[1].Fmax;
	else
		Fnorm = 0.0;
	Unit->fb[3] = kF * Fnorm;
	// -------------------------------------------------------------------------------------------------------------------------------------------
*/

	// �r���א��� --------------------------------------------------------------------------------------------------------------------------------
	// 6�ԋ؂̋ؒ��X�V
	if(x == LF || x == RF) // �O�r
//		Unit->Muscle_Len[5] = Calc_Muscle6_F_len(enc->knee_deg[x], enc->ankle_deg[x]);
		Muscle_Len = Calc_Muscle6_F_len(enc->angle_knee_deg[x], enc->angle_ankle_deg[x]);
	else	// ��r
//		Unit->Muscle_Len[5] = Calc_Muscle6_H_len(enc->knee_deg[x], enc->ankle_deg[x]);
		Muscle_Len = Calc_Muscle6_H_len(enc->angle_knee_deg[x], enc->angle_ankle_deg[x]);

	// CPG�ʑ������i�x���r���j���r�ڒn���̂݋r���׃t�B�[�h�o�b�N��K�p
	if (fV[RG_E]-fV[RG_F] > 0.0 && load->touch[x] != 0) {
		// �r���א���(�l�H�ؓ��������)
		// �r���� = ���͋C������ * �l�H�ؓ��̐L����
		// 	�l�H�ؓ��̐L���� = (���݋ؒ� - �ŏ��ؒ�) / �ő�ؒ�	�� �ŏ��ؒ�/�ő�ؒ� = 0.66
		if(x == LF || x == RF) {  // �O�r
//			load->load_value[x] = load_gain_F * load->load_char[x] * (Unit->Muscle[5].dL / Elbow_Extensor_Length_MAX - 0.66);
//			load->load_value[x] = load_gain_F * load->load_char[x] * (Muscle_Len * 0.006667 - 0.66);
//			load->load_value[x] = load_gain_F * load->load_char[x] * ((Muscle_Len-60) * 0.01111 - 0.66);
			load->load_value[x] = load_gain_F * load->load_char[x] * (Muscle_Len * 0.00671 - 0.804);
//			printf("%f\n", load->load_value[x]);

		}
		else {	// ��r
//			load->load_value[x] = load_gain_H * load->load_char[x] * (Unit->Muscle[5].dL / Ankle_Extensor_Length_MAX - 0.66);
//			load->load_value[x] = load_gain_H * load->load_char[x] * (Muscle_Len * 0.005556 - 0.66);
//			load->load_value[x] = load_gain_H * load->load_char[x] * ((Muscle_Len-60) * 0.00833 - 0.66);
			load->load_value[x] = load_gain_H * load->load_char[x] * (Muscle_Len * 0.00558 - 0.786);

		}
	}
	else
		load->load_value[x] = 0.0;

	feed_prs = 1.0 / ( 1.0 + exp( -(load->load_value[x] - 2.0/*5.0*/) * sigmoid_alpha ) );

  /*if(feed_prs < 0.5)//�r����0�̂Ƃ��V�O���C�h���W���ɂ����0�ɗ����Ȃ��̂�臒l���Ⴂ�Ƃ��͐؂�̂Ă�
		feed_prs = 0.0;*/
	// -------------------------------------------------------------------------------------------------------------------------------------------

	// �p�����[�^����
	// type1: RG_F, RG_E, PF_Sw, PF_St, PF_Lo, PF_Td, Mn_1, Mn_2, Mn_3, Mn_4, Mn_5, Mn_6
	for (i=0; i<DEF_C; i++) {
//		mK[i] = 1.0 / (1.0 + exp( -(y[i] + 44.5) * 0.2) );
		mK[i] = Calc_mK(i_V[i]);
//		mNaP[i] = 1.0 / (1.0 + exp( -(y[i] + 47.1)/3.1) );
		mNaP[i] = Calc_mNaP(i_V[i]);
//		hInfNaP[i] = 1.0 / (1.0 + exp( (y[i] + 51.0)*0.25) );
		hInfNaP[i] = Calc_hInfNaP(i_V[i]);

		if (i==0 || i==1) { // RG
			INaP[i] = _gNaP_RG * mNaP[i] * y[i+DEF_B] * (y[i] - ENa);
//			tauhNaP[i] = tauhNaPmax_RG / cosh( (y[i] + 51.0)*0.125 );
			tauhNaP_inv[i] = Calc_tauhNaP_RG_inv(i_V[i]);
		}
		else if (PF_NEURONS(i)) { // PF
			INaP[i] = _gNaP_PF * mNaP[i] * y[i+DEF_B] * (y[i] - ENa);
//			tauhNaP[i] = tauhNaPmax / cosh( (y[i] + 51.0)*0.125 );
			tauhNaP_inv[i] = Calc_tauhNaP_inv(i_V[i]);
		}
		else if (MN_NEURONS(i)) { // Mn
			INaP[i] = _gNaP_Mn * mNaP[i] * y[i+DEF_B] * (y[i] - ENa);
//			tauhNaP[i] = tauhNaPmax / cosh( (y[i] + 51.0)*0.125 );
			tauhNaP_inv[i] = Calc_tauhNaP_inv(i_V[i]);
		}

		IK[i] = _gK * pow4(mK[i]) * (y[i] - EK);
		ILeak[i] = _gLeak * (y[i] - ELeak);

		// ����������
		ISynE[i] = _gSynE * (y[i] - ESynE)
						* (	(MATa[0][i]*fV[RG_F] + MATa[1][i]*fV[RG_E] + MATa[2][i]*fV[PF_Sw] + MATa[3][i]*fV[PF_St] + MATa[4][i]*fV[Inab_E] + MATa[5][i]*fV[PF_Lo] + MATa[6][i]*fV[PF_Td] + MATa[7][i]*fV[Fb_1] + MATa[8][i]*fV[Fb_2] + MATa[9][i]*fV[Fb_3] + MATa[10][i]*fV[Fb_4] + MATa[11][i]*feed_prs + MATb[7][i] * eF + MATb[8][i] * eE)
							+ (MATc[0][i]*d[0] + MATc[1][i]*d[1])
							+ (fb[0]*MATw[0][i] + fb[1]*MATw[1][i] + fb[2]*MATw[2][i] + fb[3]*MATw[3][i])
              						+ (fb_trig*MATw[4][i]) );//hip�g���K

		// �}��������
		ISynI[i] = _gSynI * (y[i] - ESynI) * (MATb[0][i]*fV[In_F] + MATb[1][i]*fV[In_E] + MATb[2][i]*fV[In] + MATb[3][i]*fV[In_PF_Sw] + MATb[4][i]*fV[In_PF_St] + MATb[5][i]*fV[In_PF_Lo] + MATb[6][i]*fV[In_PF_Td] + MATb[7][i]*iF + MATb[8][i]*iE + MATb[9][i]*fV[In_Feed]);//20191126�@feed_prs��In_Feed�ɕύX
	}

	// type2: In_F, In_E, In, Inab-E, In_PF-Sw, In_PF-St, In_PF-Lo, In_PF-Td, Fb_1, Fb_2, Fb_3, Fb_4
	for (i=DEF_C; i<DEF_B; i++) {
		ILeak[i] = _gLeak * (y[i] - ELeakIn);

		ISynE[i] = _gSynE * (y[i] - ESynE)
						* (	(MATa[0][i]*fV[RG_F] + MATa[1][i]*fV[RG_E] + MATa[2][i]*fV[PF_Sw] + MATa[3][i]*fV[PF_St] + MATa[4][i]*fV[Inab_E] + MATa[5][i]*fV[PF_Lo] + MATa[6][i]*fV[PF_Td] + MATa[7][i]*fV[Fb_1] + MATa[8][i]*fV[Fb_2] + MATa[9][i]*fV[Fb_3] + MATa[10][i]*fV[Fb_4] + MATa[11][i]*feed_prs)
							+ (MATc[0][i]*d[0] + MATc[1][i]*d[1])
							+ (fb[0]*MATw[0][i] + fb[1]*MATw[1][i] + fb[2]*MATw[2][i] + fb[3]*MATw[3][i])
              						+ (fb_trig*MATw[4][i]) );//hip�g���K

		ISynI[i] = _gSynI * (y[i] - ESynI) * (MATb[0][i]*fV[In_F] + MATb[1][i]*fV[In_E] + MATb[2][i]*fV[In] + MATb[3][i]*fV[In_PF_Sw] + MATb[4][i]*fV[In_PF_St] + MATb[5][i]*fV[In_PF_Lo] + MATb[6][i]*fV[In_PF_Td] + MATb[7][i]*iF + MATb[8][i]*iE + MATb[9][i]*fV[In_Feed]);//20191126�@feed_prs��In_Feed�ɕύX
	}

	// �����������L�q
	for (i=0; i<DEF_C; i++)
		f[i] = (-INaP[i] - IK[i] - ILeak[i] - ISynE[i] - ISynI[i]) * 5.0e+7;
	for (i=DEF_C; i<DEF_B; i++)
		f[i] = (-ILeak[i] - ISynE[i] - ISynI[i]) * 5.0e+7;
	for (i=DEF_B; i<DEF_A; i++)
//		f[i] = (hInfNaP[i-DEF_B] - y[i]) / tauhNaP[i-DEF_B];
		f[i] = (hInfNaP[i-DEF_B] - y[i]) * tauhNaP_inv[i-DEF_B];
}


// 2�������Q�N�b�^
void CalcUnit_RK2(RybakUnit Unit[UNITS], float d[2], float h, ENCType *enc, LOADType *load)
{
	int	i, j;
	float	k1[UNITS][DEF_A], k2[UNITS][DEF_A];
	float	ya[UNITS][DEF_A], y0[UNITS][DEF_A], f[UNITS][DEF_A], y[UNITS][DEF_A];
//	float	xa;
	float	iF[UNITS], iE[UNITS], eF[UNITS], eE[UNITS];
	int	i_V;

	// y��Unit�̊֌W�\--------------------------------------------
	// V
	//	y[i][0]  : Unit[i]->Neuron[0].V		//RG-F
	//	y[i][1]  : Unit[i]->Neuron[1].V		//RG-E
	//	y[i][2]  : Unit[i]->Neuron[2].V		//PF-Sw
	//	y[i][3]  : Unit[i]->Neuron[3].V		//PF-St
	//	y[i][4]  : Unit[i]->Neuron[4].V		//PF-Lo
	//	y[i][5]  : Unit[i]->Neuron[5].V		//PF-Td
	//	y[i][6]  : Unit[i]->Neuron[6].V		//Mn-1
	//	y[i][7]  : Unit[i]->Neuron[7].V		//Mn-2
	//	y[i][8]  : Unit[i]->Neuron[8].V		//Mn-3
	//	y[i][9]  : Unit[i]->Neuron[9].V		//Mn-4
	//	y[i][10]  : Unit[i]->Neuron[10].V	//Mn-5
	//	y[i][11]  : Unit[i]->Neuron[11].V	//Mn-6
	//	y[i][12]  : Unit[i]->Neuron[12].V	//In_F
	//	y[i][13]  : Unit[i]->Neuron[13].V	//In_E
	//	y[i][14]  : Unit[i]->Neuron[14].V	//In
	//	y[i][15]  : Unit[i]->Neuron[15].V	//Inab_E
	//	y[i][16]  : Unit[i]->Neuron[16].V	//In_PF-Sw
	//	y[i][17]  : Unit[i]->Neuron[17].V	//In_PF-St
	//	y[i][18]  : Unit[i]->Neuron[18].V	//In_PF-Lo
	//	y[i][19]  : Unit[i]->Neuron[19].V	//In_PF-Td
	//	y[i][20]  : Unit[i]->Neuron[20].V	//Fb_1
	//	y[i][21]  : Unit[i]->Neuron[21].V	//Fb_2
	//	y[i][22]  : Unit[i]->Neuron[22].V	//Fb_3
	//	y[i][23]  : Unit[i]->Neuron[23].V	//Fb_4
	// hNaP
	//	y[i][24] : Unit[i]->Neuron[0].hNaP	//RG-F
	//	y[i][25] : Unit[i]->Neuron[1].hNaP	//RG-E
	//	y[i][26] : Unit[i]->Neuron[2].hNaP	//PF-Sw
	//	y[i][27] : Unit[i]->Neuron[3].hNaP	//PF-St
	//	y[i][28] : Unit[i]->Neuron[4].hNaP	//PF-Lo
	//	y[i][29] : Unit[i]->Neuron[5].hNaP	//PF-Td
	//	y[i][30] : Unit[i]->Neuron[6].hNaP	//Mn-1
	//	y[i][31] : Unit[i]->Neuron[7].hNaP	//Mn-2
	//	y[i][32] : Unit[i]->Neuron[8].hNaP	//Mn-3
	//	y[i][33] : Unit[i]->Neuron[9].hNaP	//Mn-4
	//	y[i][34] : Unit[i]->Neuron[10].hNaP	//Mn-5
	//	y[i][35] : Unit[i]->Neuron[11].hNaP	//Mn-6
	// -----------------------------------------------------------

	// Unit���̊e�j���[������V�y��hNaP��y�֊i�[
	for (i=0; i<UNITS; i++) {
		for (j=0; j<DEF_B; j++)
			y[i][j] = Unit[i].Neuron_V[j];
		for (j=DEF_B; j<DEF_A; j++)
			y[i][j] = Unit[i].Neuron_hNaP[j-DEF_B];
	}

	// RK2 Calculation
	// k1�����߂� ------------------------------------------------------------------------------
	for (i=0; i<UNITS; i++) {
	        for ( j=0 ; j<DEF_A ; j++ )
			y0[i][j]=y[i][j];
	}
	// ���j�b�g�Ԍ���
	for (i=0; i<UNITS; i++) {
		iF[i] = 0.0;
		iE[i] = 0.0;
		eF[i] = 0.0;
		eE[i] = 0.0;
		
	}

	for (i=0; i<UNITS; i++) {
		for (j=0; j<UNITS; j++) {
//			i_V = (int)( (y[j][RG_F] - V_MIN) / (V_MAX - V_MIN) * (V_num - 1) );
			i_V = (int)( (y[j][RG_F] + 80.0) * 9.090909 + 0.00001);	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001�̎�
//			iF[i] += GetfV(y[j][RG_F], RG_F) * c_F[i][j];
			iF[i] += Calc_fV(i_V) * c_F[i][j];
			eF[i] += Calc_fV(i_V) * b_F[i][j];

//			i_V = (int)( (y[j][RG_E] - V_MIN) / (V_MAX - V_MIN) * (V_num - 1) );
			i_V = (int)( (y[j][RG_E] + 80.0) * 9.090909 + 0.00001);	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001�̎�
//			iE[i] += GetfV(y[j][RG_E], RG_E) * c_E[i][j];
			iE[i] += Calc_fV(i_V) * c_E[i][j];
			eE[i] += Calc_fV(i_V) * b_E[i][j];
		}
	}
	for (i=0; i<UNITS; i++) {
		if (i==0 || i==2)	// �O�r
			aux(i, y[i], f[i], iF[i], iE[i], eF[i], eE[i],d, h, MATc_fore, MATa_fore, MATb_fore, MATw_fore, enc, load);
		else // ��r
			aux(i, y[i], f[i], iF[i], iE[i], eF[i], eE[i],d, h, MATc_hind, MATa_hind, MATb_hind, MATw_hind, enc, load);
	}

	for (i=0; i<UNITS; i++) {
		// V�̐���
		for (j=0; j<DEF_B; j++) {
			k1[i][j] = h * f[i][j];
			ya[i][j] = y0[i][j] + k1[i][j];
			if (ya[i][j] < V_MIN) ya[i][j] = V_MIN;
			if (ya[i][j] > V_MAX) ya[i][j] = V_MAX;
		}
		// hNaP�̐���
		for (j=DEF_B; j<DEF_A; j++) {
			k1[i][j] = h * f[i][j];
			ya[i][j] = y0[i][j] + k1[i][j];
		}
	}
	// -----------------------------------------------------------------------------------------
	// k2�����߂� ------------------------------------------------------------------------------
//	xa=x+h;
	// ���j�b�g�Ԍ���
	for (i=0; i<UNITS; i++) {
		iF[i] = 0.0;
		iE[i] = 0.0;
		eF[i] = 0.0;
		eE[i] = 0.0;
		
	}
	for (i=0; i<UNITS; i++) {
		for (j=0; j<UNITS; j++) {
//			i_V = (int)( (ya[j][RG_F] - V_MIN) / (V_MAX - V_MIN) * (V_num - 1) );
			i_V = (int)( (ya[j][RG_F] + 80.0) * 9.090909 + 0.00001);	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001�̎�
//			iF[i] += GetfV(ya[j][RG_F], RG_F) * c_F[i][j];
			iF[i] += Calc_fV(i_V) * c_F[i][j];
			eF[i] += Calc_fV(i_V) * b_F[i][j];

//			i_V = (int)( (ya[j][RG_E] - V_MIN) / (V_MAX - V_MIN) * (V_num - 1) );
			i_V = (int)( (ya[j][RG_E] + 80.0) * 9.090909 + 0.00001);	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001�̎�
//			iE[i] += GetfV(ya[j][RG_E], RG_E) * c_E[i][j];
			iE[i] += Calc_fV(i_V) * c_E[i][j];
			eE[i] += Calc_fV(i_V) * b_E[i][j];
		}
	}

	for (i=0; i<UNITS; i++) {
		if (i==0 || i==2)	// �O�r
			aux(i, ya[i], f[i], iF[i], iE[i], eF[i], eE[i], d, h, MATc_fore, MATa_fore, MATb_fore, MATw_fore, enc, load);
		else // ��r
			aux(i, ya[i], f[i], iF[i], iE[i], eF[i], eE[i], d, h, MATc_hind, MATa_hind, MATb_hind, MATw_hind, enc, load);
	}

	for (i=0; i<UNITS; i++) {
		// V�̐���
		for (j=0; j<DEF_B; j++) {
			k2[i][j] = h * f[i][j];
			y[i][j] = y0[i][j] + (k1[i][j] + k2[i][j]) * 0.5;
			if (y[i][j] < V_MIN) y[i][j] = V_MIN;
			if (y[i][j] > V_MAX) y[i][j] = V_MAX;
		}
		// hNaP�̐���
		for (j=DEF_B; j<DEF_A; j++) {
			k2[i][j] = h * f[i][j];
			y[i][j] = y0[i][j] + (k1[i][j] + k2[i][j]) * 0.5;
		}
	}
	// -----------------------------------------------------------------------------------------
	// y�̒l��Unit�֔��f���C�e�j���[������V�y��hNaP���X�V
	for (i=0; i<UNITS; i++) {
		for (j=0; j<DEF_B; j++)
			Unit[i].Neuron_V[j] = y[i][j];
		for (j=0; j<DEF_C; j++)
			Unit[i].Neuron_hNaP[j] = y[i][j+DEF_B];
	}
}

// 4�������Q�N�b�^
void CalcUnit_RK4(RybakUnit Unit[UNITS], float d[2], float h, ENCType *enc, LOADType *load)
{
	int	i, j;
	float	k1[UNITS][DEF_A], k2[UNITS][DEF_A], k3[UNITS][DEF_A], k4[UNITS][DEF_A];
	float	ya[UNITS][DEF_A], y0[UNITS][DEF_A], f[UNITS][DEF_A], y[UNITS][DEF_A];
	float	xa;
	float	iF[UNITS], iE[UNITS], eF[UNITS], eE[UNITS];
	int	i_V;

	// y��Unit�̊֌W�\--------------------------------------------
	// V
	//	y[i][0]  : Unit[i]->Neuron[0].V		//RG-F
	//	y[i][1]  : Unit[i]->Neuron[1].V		//RG-E
	//	y[i][2]  : Unit[i]->Neuron[2].V		//PF-Sw
	//	y[i][3]  : Unit[i]->Neuron[3].V		//PF-St
	//	y[i][4]  : Unit[i]->Neuron[4].V		//PF-Lo
	//	y[i][5]  : Unit[i]->Neuron[5].V		//PF-Td
	//	y[i][6]  : Unit[i]->Neuron[6].V		//Mn-1
	//	y[i][7]  : Unit[i]->Neuron[7].V		//Mn-2
	//	y[i][8]  : Unit[i]->Neuron[8].V		//Mn-3
	//	y[i][9]  : Unit[i]->Neuron[9].V		//Mn-4
	//	y[i][10]  : Unit[i]->Neuron[10].V	//Mn-5
	//	y[i][11]  : Unit[i]->Neuron[11].V	//Mn-6
	//	y[i][12]  : Unit[i]->Neuron[12].V	//In_F
	//	y[i][13]  : Unit[i]->Neuron[13].V	//In_E
	//	y[i][14]  : Unit[i]->Neuron[14].V	//In
	//	y[i][15]  : Unit[i]->Neuron[15].V	//Inab_E
	//	y[i][16]  : Unit[i]->Neuron[16].V	//In_PF-Sw
	//	y[i][17]  : Unit[i]->Neuron[17].V	//In_PF-St
	//	y[i][18]  : Unit[i]->Neuron[18].V	//In_PF-Lo
	//	y[i][19]  : Unit[i]->Neuron[19].V	//In_PF-Td
	//	y[i][20]  : Unit[i]->Neuron[20].V	//Fb_1
	//	y[i][21]  : Unit[i]->Neuron[21].V	//Fb_2
	//	y[i][22]  : Unit[i]->Neuron[22].V	//Fb_3
	//	y[i][23]  : Unit[i]->Neuron[23].V	//Fb_4
	// hNaP
	//	y[i][24] : Unit[i]->Neuron[0].hNaP	//RG-F
	//	y[i][25] : Unit[i]->Neuron[1].hNaP	//RG-E
	//	y[i][26] : Unit[i]->Neuron[2].hNaP	//PF-Sw
	//	y[i][27] : Unit[i]->Neuron[3].hNaP	//PF-St
	//	y[i][28] : Unit[i]->Neuron[4].hNaP	//PF-Lo
	//	y[i][29] : Unit[i]->Neuron[5].hNaP	//PF-Td
	//	y[i][30] : Unit[i]->Neuron[6].hNaP	//Mn-1
	//	y[i][31] : Unit[i]->Neuron[7].hNaP	//Mn-2
	//	y[i][32] : Unit[i]->Neuron[8].hNaP	//Mn-3
	//	y[i][33] : Unit[i]->Neuron[9].hNaP	//Mn-4
	//	y[i][34] : Unit[i]->Neuron[10].hNaP	//Mn-5
	//	y[i][35] : Unit[i]->Neuron[11].hNaP	//Mn-6
	// -----------------------------------------------------------

	// �e�j���[������V�y��hNaP��y�֊i�[
	for (i=0; i<UNITS; i++) {
		for (j=0; j<DEF_B; j++)
			y[i][j] = Unit[i].Neuron_V[j];
		for (j=DEF_B; j<DEF_A; j++)
			y[i][j] = Unit[i].Neuron_hNaP[j-DEF_B];
	}

	// RK4 Calculation
	// k1�����߂� ------------------------------------------------------------------------------
	for (i=0; i<UNITS; i++) {
	        for ( j=0 ; j<DEF_A ; j++ )
			y0[i][j]=y[i][j];	// Unit�̒l�ɃX�^�b�N����Ă邩��y0�͗v��Ȃ�
	}
	// ���j�b�g�Ԍ���
	for (i=0; i<UNITS; i++) {
		iF[i] = 0.0;
		iE[i] = 0.0;
		eF[i] = 0.0;
		eE[i] = 0.0;
	}
	for (i=0; i<UNITS; i++) {
		for (j=0; j<UNITS; j++) {
//			i_V = (int)( (y[j][RG_F] - V_MIN) / (V_MAX - V_MIN) * (V_num - 1) );
			i_V = (int)( (y[j][RG_F] + 80.0) * 9.090909 + 0.00001);	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001�̎�
//			iF[i] += GetfV(y[j][RG_F], RG_F) * c_F[i][j];
			iF[i] += Calc_fV(i_V) * c_F[i][j];
			eF[i] += Calc_fV(i_V) * b_F[i][j];

//			i_V = (int)( (y[j][RG_E] - V_MIN) / (V_MAX - V_MIN) * (V_num - 1) );
			i_V = (int)( (y[j][RG_E] + 80.0) * 9.090909 + 0.00001);	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001�̎�
//			iE[i] += GetfV(y[j][RG_E], RG_E) * c_E[i][j];
			iE[i] += Calc_fV(i_V) * c_E[i][j];
			eE[i] += Calc_fV(i_V) * b_E[i][j];
		}
	}
	for (i=0; i<UNITS; i++) {
		if (i==0 || i==2)	// �O�r
			aux(i, y[i], f[i], iF[i], iE[i], eF[i], eE[i], d, h, MATc_fore, MATa_fore, MATb_fore, MATw_fore, enc, load);
		else // ��r
			aux(i, y[i], f[i], iF[i], iE[i], eF[i], eE[i], d, h, MATc_hind, MATa_hind, MATb_hind, MATw_hind, enc, load);
	}

	for (i=0; i<UNITS; i++) {
		// V�̐���
		for (j=0; j<DEF_B; j++) {
			k1[i][j] = h * f[i][j];
			ya[i][j] = y0[i][j] + (0.5 * k1[i][j]);
			if (ya[i][j] < V_MIN) ya[i][j] = V_MIN;
			if (ya[i][j] > V_MAX) ya[i][j] = V_MAX;
		}
		// hNaP�̐���
		for (j=DEF_B; j<DEF_A; j++) {
			k1[i][j] = h * f[i][j];
			ya[i][j] = y0[i][j] + (0.5 * k1[i][j]);
		}
	}
	// -----------------------------------------------------------------------------------------
	// k2�����߂� ------------------------------------------------------------------------------
//	xa=x+0.5*h;
	// ���j�b�g�Ԍ���
	for (i=0; i<UNITS; i++) {
		iF[i] = 0.0;
		iE[i] = 0.0;
		eF[i] = 0.0;
		eE[i] = 0.0;
	}
	for (i=0; i<UNITS; i++) {
		for (j=0; j<UNITS; j++) {
//			i_V = (int)( (ya[j][RG_F] - V_MIN) / (V_MAX - V_MIN) * (V_num - 1) );
			i_V = (int)( (ya[j][RG_F] + 80.0) * 9.090909 + 0.00001);	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001�̎�
//			iF[i] += GetfV(ya[j][RG_F], RG_F) * c_F[i][j];
			iF[i] += Calc_fV(i_V) * c_F[i][j];
			eF[i] += Calc_fV(i_V) * b_F[i][j];

//			i_V = (int)( (ya[j][RG_E] - V_MIN) / (V_MAX - V_MIN) * (V_num - 1) );
			i_V = (int)( (ya[j][RG_E] + 80.0) * 9.090909 + 0.00001);	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001�̎�
//			iE[i] += GetfV(ya[j][RG_E], RG_E) * c_E[i][j];
			iE[i] += Calc_fV(i_V) * c_E[i][j];
			eE[i] += Calc_fV(i_V) * b_E[i][j];
		}
	}
	for (i=0; i<UNITS; i++) {
		if (i==0 || i==2)	// �O�r
			aux(i, ya[i], f[i], iF[i], iE[i], eF[i], eE[i], d, h, MATc_fore, MATa_fore, MATb_fore, MATw_fore, enc, load);
		else // ��r
			aux(i, ya[i], f[i], iF[i], iE[i], eF[i], eE[i], d, h, MATc_hind, MATa_hind, MATb_hind, MATw_hind, enc, load);
	}
	for (i=0; i<UNITS; i++) {
		// V�̐���
		for (j=0; j<DEF_B; j++) {
			k2[i][j] = h * f[i][j];
			ya[i][j] = y0[i][j] + (0.5 * k2[i][j]);
			if (ya[i][j] < V_MIN) ya[i][j] = V_MIN;
			if (ya[i][j] > V_MAX) ya[i][j] = V_MAX;
		}
		// hNaP�̐���
		for (j=DEF_B; j<DEF_A; j++) {
			k2[i][j] = h * f[i][j];
			ya[i][j] = y0[i][j] + (0.5 * k2[i][j]);
		}
	}
	// -----------------------------------------------------------------------------------------
	// k3�����߂� ------------------------------------------------------------------------------
	// ���j�b�g�Ԍ���
	for (i=0; i<UNITS; i++) {
		iF[i] = 0.0;
		iE[i] = 0.0;
		eF[i] = 0.0;
		eE[i] = 0.0;
	}
	for (i=0; i<UNITS; i++) {
		for (j=0; j<UNITS; j++) {
//			i_V = (int)( (ya[j][RG_F] - V_MIN) / (V_MAX - V_MIN) * (V_num - 1) );
			i_V = (int)( (ya[j][RG_F] + 80.0) * 9.090909 + 0.00001);	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001�̎�
//			iF[i] += GetfV(ya[j][RG_F], RG_F) * c_F[i][j];
			iF[i] += Calc_fV(i_V) * c_F[i][j];
			eF[i] += Calc_fV(i_V) * b_F[i][j];

//			i_V = (int)( (ya[j][RG_E] - V_MIN) / (V_MAX - V_MIN) * (V_num - 1) );
			i_V = (int)( (ya[j][RG_E] + 80.0) * 9.090909 + 0.00001);	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001�̎�
//			iE[i] += GetfV(ya[j][RG_E], RG_E) * c_E[i][j];
			iE[i] += Calc_fV(i_V) * c_E[i][j];
			eE[i] += Calc_fV(i_V) * b_E[i][j];
		}
	}
	for (i=0; i<UNITS; i++) {
		if (i==0 || i==2)	// �O�r
			aux(i, ya[i], f[i], iF[i], iE[i], eF[i], eE[i], d, h, MATc_fore, MATa_fore, MATb_fore, MATw_fore, enc, load);
		else // ��r
			aux(i, ya[i], f[i], iF[i], iE[i], eF[i], eE[i], d, h, MATc_hind, MATa_hind, MATb_hind, MATw_hind, enc, load);
	}
	for (i=0; i<UNITS; i++) {
		// V�̐���
		for (j=0; j<DEF_B; j++) {
			k3[i][j] = h * f[i][j];
			ya[i][j] = y0[i][j] + k3[i][j];
			if (ya[i][j] < V_MIN) ya[i][j] = V_MIN;
			if (ya[i][j] > V_MAX) ya[i][j] = V_MAX;
		}
		// hNaP�̐���
		for (j=DEF_B; j<DEF_A; j++) {
			k3[i][j] = h * f[i][j];
			ya[i][j] = y0[i][j] + k3[i][j];
		}
	}
	// -----------------------------------------------------------------------------------------
	// k4�����߂� ------------------------------------------------------------------------------
//	xa=x+h;
	// ���j�b�g�Ԍ���
	for (i=0; i<UNITS; i++) {
		iF[i] = 0.0;
		iE[i] = 0.0;
		eF[i] = 0.0;
		eE[i] = 0.0;
	}
	for (i=0; i<UNITS; i++) {
		for (j=0; j<UNITS; j++) {
//			i_V = (int)( (ya[j][RG_F] - V_MIN) / (V_MAX - V_MIN) * (V_num - 1) );
			i_V = (int)( (ya[j][RG_F] + 80.0) * 9.090909 + 0.00001);	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001�̎�
//			iF[i] += GetfV(ya[j][RG_F], RG_F) * c_F[i][j];
			iF[i] += Calc_fV(i_V) * c_F[i][j];
			eF[i] += Calc_fV(i_V) * b_F[i][j];

//			i_V = (int)( (ya[j][RG_E] - V_MIN) / (V_MAX - V_MIN) * (V_num - 1) );
			i_V = (int)( (ya[j][RG_E] + 80.0) * 9.090909 + 0.00001);	// V_MAX = 30.0, V_MIN = -80.0, V_num = 1001�̎�
//			iE[i] += GetfV(ya[j][RG_E], RG_E) * c_E[i][j];
			iE[i] += Calc_fV(i_V) * c_E[i][j];
			eE[i] += Calc_fV(i_V) * b_E[i][j];
		}
	}
	for (i=0; i<UNITS; i++) {
		if (i==0 || i==2)	// �O�r
			aux(i, ya[i], f[i], iF[i], iE[i], eF[i], eE[i], d, h, MATc_fore, MATa_fore, MATb_fore, MATw_fore, enc, load);
		else // ��r
			aux(i, ya[i], f[i], iF[i], iE[i], eF[i], eE[i], d, h, MATc_hind, MATa_hind, MATb_hind, MATw_hind, enc, load);
	}
	for (i=0; i<UNITS; i++) {
		// V�̐���
		for (j=0; j<DEF_B; j++) {
			k4[i][j] = h * f[i][j];
			y[i][j] = y0[i][j] + (k1[i][j] + (k2[i][j]+k3[i][j])*2.0 + k4[i][j]) * 0.166667;
			if (y[i][j] < V_MIN) y[i][j] = V_MIN;
			if (y[i][j] > V_MAX) y[i][j] = V_MAX;
		}
		// hNaP�̐���
		for (j=DEF_B; j<DEF_A; j++) {
			k4[i][j] = h * f[i][j];
			y[i][j] = y0[i][j] + (k1[i][j] + (k2[i][j]+k3[i][j])*2.0 + k4[i][j]) * 0.166667;
		}
	}
	// -----------------------------------------------------------------------------------------
	// y�̒l��Unit�֔��f���C�e�j���[������V�y��hNaP���X�V
	for (i=0; i<UNITS; i++) {
		for (j=0; j<DEF_B; j++)
			Unit[i].Neuron_V[j] = y[i][j];
		for (j=0; j<DEF_C; j++)
			Unit[i].Neuron_hNaP[j] = y[i][j+DEF_B];
	}
}


// CPG�����ERG-F���Ί��ԁERG-E���Ί��Ԃ̌v�Z
void CalcCycle(RybakUnit Unit[UNITS], float t)
{
	int	i;
	float	RG_F_fV;
	float	RG_E_fV;

	for(i=0; i<UNITS; i++){
		RG_F_fV = GetfV(Unit[i].Neuron_V[RG_F], RG_F);
		RG_E_fV = GetfV(Unit[i].Neuron_V[RG_E], RG_E);

		// CPG�����̌v�Z----------------------------------------------------------
		// CPG�����̍X�V�iRG_F�����΂������^�C�~���O�̊Ԋu��1�����Ƃ���j
		if (Unit[i].cycle_flag == 0 && RG_F_fV > 0) {
			Unit[i].cpg_cycle = t - Unit[i].timing_CPG;
			Unit[i].timing_CPG = t;
		}

		// CPG�����v�Z�p�t���O�̍X�V
		if (RG_F_fV > 0)	// RG-F�j���[���������΂��Ă���ꍇ
			Unit[i].cycle_flag = 1;
		else			// RG-F�j���[���������΂��Ă��Ȃ��ꍇ
			Unit[i].cycle_flag = 0;
		// -----------------------------------------------------------------------

		// RG-F���Ί��Ԃ̌v�Z---------------------------------------------------------
		// RG-F���ΊJ�n�����̍X�V
		if (Unit[i].F_period_flag == 0 && RG_F_fV > 0)
			Unit[i].timing_RG_F = t;

		// RG-F���Ί��Ԃ̍X�V
		if (Unit[i].F_period_flag == 1 && RG_F_fV == 0)
			Unit[i].RG_F_period = t - Unit[i].timing_RG_F;

		// RG-F���Ί��Ԍv�Z�p�t���O�̍X�V
		if (RG_F_fV > 0)	// RG-F�j���[���������΂��Ă���ꍇ
			Unit[i].F_period_flag = 1;
		else			// RG-F�j���[���������΂��Ă��Ȃ��ꍇ
			Unit[i].F_period_flag = 0;
		// -----------------------------------------------------------------------

		// RG-E���Ί��Ԃ̌v�Z-------------------------------------------------------
		// RG-E���ΊJ�n�����̍X�V
		if (Unit[i].E_period_flag == 0 && RG_E_fV > 0)
			Unit[i].timing_RG_E = t;

		// RG-E���Ί��Ԃ̍X�V
		if (Unit[i].E_period_flag == 1 && RG_E_fV == 0)
			Unit[i].RG_E_period = t - Unit[i].timing_RG_E;

		// RG-E���Ί��Ԍv�Z�p�t���O�̍X�V
		if (RG_E_fV > 0)	// RG-E�j���[���������΂��Ă���ꍇ
			Unit[i].E_period_flag = 1;
		else			// RG-E�j���[���������΂��Ă��Ȃ��ꍇ
			Unit[i].E_period_flag = 0;
		// -----------------------------------------------------------------------
	}
}
