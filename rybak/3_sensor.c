// -------------------------------------------- //
// 	�Z���T���擾�Ɋւ���֐��Q		//
//		sensor.c			//
// -------------------------------------------- //

#include "8_define.h"
#include "9_user_param.h"

#define ENC_RESOLUTION	1024	// �G���R�[�_�p���X��
#define	MP	2.0		// ���{
#define	LP_Tr	50.0		// �g���b�h�~�����x�␳�p�萔
#define	LP_FF	10.0		// FlexiForce�␳�p�萔

#define GYRO_SENS		(0.67)	// �W���C���Z���T�̊��x[mV/deg/sec]
#define GYRO_AMP		(3.0)	// �W���C���Z���T�̑����x(5.0/0.88?)
#define GYRO_SENS_AMP		(GYRO_SENS * GYRO_AMP)	//�W���C���Z���T�̑�����̊��x[mV/deg/sec]

#define ACC_SENS		(1.0)	// �����x�Z���T�̊��x[V/G](1G = 9.8m/s^2)
#define ACC_AMP			(1.0)	// �����x�Z���T�̑����x


int ENC_HIP_CH[UNITS] = {0, 4, 8, 12};		// ENC�`�����l���ԍ�(��/���)
int ENC_KNEE_CH[UNITS] = {1, 5, 9, 13};	// ENC�`�����l���ԍ�(�G/��)
int ENC_ANKLE_CH[UNITS] = {2, 6, 10, 14}; 	// ENC�`�����l���ԍ�(����/�I)
int ENC_TREADMILL_CH = 15;			// ENC�`�����l���ԍ�(�g���b�h�~��)

int AD_FF_CH[UNITS] = {1, 4, 8, 12};		// AD�`�����l���ԍ�(FlexiForce)
int AD_POT_CH[UNITS] = {0, 5, 9, 13};		// AD�`�����l���ԍ�(�|�e���V�����[�^)
int AD_GYRO_CH[num_AXIS] = {2, 14};		// AD�`�����l���ԍ�(�W���C���Z���T)
int AD_ACC_CH[num_AXIS] = {10, 6};		// AD�`�����l���ԍ�(�����x�Z���T)
int AD_AIR_PRESS_CH = 15;			// AD�`�����l���ԍ�(��C���Z���T)

/*
//------------------------------//
//	�e�֐ߊp�x�̎擾	//
//	angle_get()		//
//------------------------------//
void angle_get(int i, ENCType *enc)
{
	//�G���R�[�_�l���擾
	getEnc(ENC_HIP_CH[i], &(enc->data_hip[i]));		//��(���b��)�֐�
	getEnc(ENC_KNEE_CH[i], &(enc->data_knee[i]));		//�G(��)�֐�
	getEnc(ENC_ANKLE_CH[i], &(enc->data_ankle[i]));		//����(�I)�֐�

	//�ЂƂO�̊p�x[rad]���X�V
	enc->angle_hip_rad_last[i] = enc->angle_hip_rad[i];	//��(���b��)�֐�
	enc->angle_knee_rad_last[i] = enc->angle_knee_rad[i];	//�G(��)�֐�
	enc->angle_ankle_rad_last[i] = enc->angle_ankle_rad[i];	//����(�I)�֐�

	//�G���R�[�_�l���p�x[rad]�֕ϊ�
	enc->angle_hip_rad[i] = (float)((2.0 * PI * enc->data_hip[i]) / (ENC_RESOLUTION * MP));		//��(���b��)�֐�
	enc->angle_knee_rad[i] = (float)((2.0 * PI * enc->data_knee[i]) / (ENC_RESOLUTION * MP));	//�G(��)�֐�
	enc->angle_ankle_rad[i] = (float)((2.0 * PI * enc->data_ankle[i]) / (ENC_RESOLUTION * MP));	//����(�I)�֐�

	//��]������v(�E���r�̓G���R�[�_���t�Ɏ��t�����Ă��邽��)
	if(i == RF || i == RH){
		enc->angle_hip_rad[i] *= -1.0;		//��(���b��)�֐�
		enc->angle_knee_rad[i] *= -1.0;		//�G(��)�֐�
		enc->angle_ankle_rad[i] *= -1.0;	//����(�I)�֐�
	}

	//�p�x[deg]
	enc->angle_hip_deg[i] = enc->angle_hip_rad[i] * radtodeg;	//��(���b��)�֐�
	enc->angle_knee_deg[i] = enc->angle_knee_rad[i] * radtodeg;	//�G(��)�֐�
	enc->angle_ankle_deg[i] = enc->angle_ankle_rad[i] * radtodeg;	//����(�I)�֐�
}
*/

// ---------------------------- //
// 	4�r���̊e�֐ߊp�x�擾	//
//	all_angle_get()		//
// ---------------------------- //
void all_angle_get(ENCType *enc)
{
	int i;
	int data_hip, data_knee, data_ankle;

	// �e�֐� -------------------------------------------------------------------------------------------------
	//�G���R�[�_�l���p�x[deg]�֕ϊ�
	for(i=0; i<UNITS; i++) {	// 1 / (ENC_RESOLUTION * MP) �� 0.0004883
		// �G���R�[�_�l���擾
		getEnc(ENC_HIP_CH[i], &data_hip);		//��(���b��)�֐�
		getEnc(ENC_KNEE_CH[i], &data_knee);		//�G(��)�֐�
		getEnc(ENC_ANKLE_CH[i], &data_ankle);		//����(�I)�֐�

		// �p�x[deg]�ɕϊ��i���ΓI�Ȃ��́j
		data_hip = (float)((360.0 * data_hip) * 0.0004883);	//��(���b��)�֐�
		data_knee = (float)((360.0 * data_knee) * 0.0004883);	//�G(��)�֐�
		data_ankle = (float)((360.0 * data_ankle) * 0.0004883);	//����(�I)�֐�

		// ��]������v(�E���r�̓G���R�[�_���t�Ɏ��t�����Ă��邽��)
		if(i == LF || i == LH){
			data_hip *= -1.0;		//��(���b��)�֐�
			data_knee *= -1.0;		//�G(��)�֐�
			data_ankle *= -1.0;	//����(�I)�֐�
		}

		// �ŏI�I�Ȋp�x[deg]���擾�i��ΓI�Ȃ��́j
		if (i == LF || i == RF) {	// �O�r
			enc->angle_hip_deg[i] = data_hip + ANGLE_SCAPULA_MIN;		// ���b���֐�
			enc->angle_knee_deg[i] = data_knee + ANGLE_SHOULDER_MIN;	// ���֐�
			enc->angle_ankle_deg[i] = data_ankle + ANGLE_ELBOW_MIN;		// �I�֐�
		}
		else {		// ��r
			enc->angle_hip_deg[i] = data_hip + ANGLE_HIP_MIN;		// ���֐�
			enc->angle_knee_deg[i] = data_knee + ANGLE_KNEE_MIN;		// �G�֐�
			enc->angle_ankle_deg[i] = data_ankle + ANGLE_ANKLE_MIN;		// ����֐�
		}
	}
}


//--------------------------------------//
//	�g���b�h�~�����x�̎擾		//
//	speed_treadmill_get()		//
//--------------------------------------//
void speed_treadmill_get(ENCType *enc, float h)
{
	int data_treadmill;
	float angle_treadmill;
	float speed_treadmill;

	// �G���R�[�_�l���擾
	getEnc(ENC_TREADMILL_CH, &data_treadmill);
	// �G���R�[�_�l���p�x[rad]�֕ϊ�
	angle_treadmill = (float)((2.0 * PI * data_treadmill) * 0.0004883);	// �ϊ�
	// �g���b�h�~�����xv = ���� * r / ��t
	speed_treadmill = (angle_treadmill - enc->angle_treadmill_rad) * 0.025 / h; //* 5.0e2;// 1.0/h = 500;
	// ���[�p�X�t�B���^
//	speed_treadmill = (enc->speed_treadmill * LP_Tr + speed_treadmill) / (LP_Tr + 1.0);
	speed_treadmill = (enc->speed_treadmill * 50.0 + speed_treadmill) * 0.019608;	// LP_Tr = 50.0
	// �X�V
	enc->angle_treadmill_rad = angle_treadmill;
	enc->speed_treadmill = speed_treadmill;
}


/*
//------------------------------------------------------//
//	�^���r���׎擾(6�ԋؒ��̕ψʂ���r���א���)	//
//			load_get()			//
//------------------------------------------------------//
void load_get(int i, LOADType *load, RybakUnit *Unit)
{
	if(load->flag_load[i] == 1 && Unit->Neuron[RG_E].V > 0.0) {
		// �r���א���(�l�H�ؓ��������)
		// �r���� = ���͋C������ * �l�H�ؓ��̐L����
		// 	�l�H�ؓ��̐L���� = (���݋ؒ� - �ŏ��ؒ�) / �ő�ؒ�	�� �ŏ��ؒ�/�ő�ؒ� = 0.66�i�d�l���L�ڂ̎��k��34%���j
		if(i == LF || i == RF) {  // �O�r
//			load->load_value[i] = load->load_char[i] * (Unit->Muscle[5].dL / Elbow_Extensor_Length_MAX - 0.66);
			load->load_value[i] = load->load_char[i] * (Unit->Muscle_Len[5] * 0.006667 - 0.66);
		}
		else {	// ��r
//			load->load_value[i] = load->load_char[i] * (Unit->Muscle[5].dL / Ankle_Extensor_Length_MAX - 0.66);
			load->load_value[i] = load->load_char[i] * (Unit->Muscle_Len[5] * 0.005556 - 0.66);
		}
	}
	else if(load->flag_load[i] == 0)
		load->load_value[i] = 0.0;

	//Unit->feed_prs = load->load_value[i];
}
*/

//--------------------------------------//
//	�I�t�Z�b�g�v��(���Z�p)		//
//	measure_offset_sum()		//
//--------------------------------------//
void measure_offset_sum(ADType *ad)
{
	int	i;			// �J�E���^
	double	ad_data[num_AD];	// AD�|�[�g�o�͊i�[��

	// �SAD�|�[�g�o�͒l�̎擾
	getADall(ad_data);

	// �I�t�Z�b�g�l�����Z(�ŏI�I��measure_offset_con()�ɂĕ��ω��������̂��I�t�Z�b�g�l�Ƃ���)
	for(i=0; i<UNITS; i++)
		ad->POT_data_offset[i] += (float)ad_data[AD_POT_CH[i]];		// �|�e���V�����[�^
	ad->gyro_data_offset[pitch] += (float)ad_data[AD_GYRO_CH[pitch]];	// �W���C���Z���T�o��(pitch)
	ad->gyro_data_offset[roll] += (float)ad_data[AD_GYRO_CH[roll]];		// 	 �@�V	     (roll)
	ad->acc_data_offset[pitch] += (float)ad_data[AD_ACC_CH[pitch]];		// �����x�Z���T�o��(pitch)
	ad->acc_data_offset[roll] += (float)ad_data[AD_ACC_CH[roll]];		// 	�@�V	�@ (roll)
}

//----------------------------------------------//
//	�I�t�Z�b�g�v��(�I�t�Z�b�g����p)	//
//		measure_offset_con()		//
//----------------------------------------------//
void measure_offset_con(int k, ADType *ad)
{
	int 	i;			// �J�E���^
	float	acc_g[num_AXIS];	// �������ɂ�����d�͉����x[G]

	// measure_offset_sum()�ł̑��a�����Z������k�Ŋ��邱�Ƃŕ��ω����C�I�t�Z�b�g�l�Ƃ��Č���
	for(i=0; i<UNITS; i++) {
		ad->POT_data_offset[i] /= k;	// �|�e���V�����[�^�̏o��[V]
		// ����v���[�g�̃I�t�Z�b�g�p�x���v�Z
		if(i == LF || i == LH) {
			ad->foot_angle_offset[i] = ad->foot_angle_last[i] = ad->POT_data_offset[i] * 330 * 0.2;
		}
		else {	// RF or RH
			ad->foot_angle_offset[i] = ad->foot_angle_last[i] = 330 - (ad->POT_data_offset[i] * 330 * 0.2);
		}
	}
	ad->gyro_data_offset[pitch] /= (float)k;	// �W���C���Z���T�o��[V](pitch)
	ad->gyro_data_offset[roll] /= (float)k;		// 	   �V	�@�@ [V](roll)
	ad->acc_data_offset[pitch] /= (float)k;		// �����x�Z���T�o��[V](pitch)
	ad->acc_data_offset[roll] /= (float)k;		// 	  �V	�@ [V](roll)

	// �����x�Z���T�̃I�t�Z�b�g�o�͂��瓷�̌X���̃I�t�Z�b�g�p�x���v�Z--------------------------------
	// �o��[V]����s�b�`�������y�у��[���������ɂ������Ă���d�͉����x���Z�o(����:0G �` ����:�}1G)
	acc_g[pitch] = (ad->acc_data_offset[pitch] - (2.5 * ACC_AMP)) / (ACC_SENS * ACC_AMP);
	acc_g[roll] = (ad->acc_data_offset[roll] - (2.5 * ACC_AMP)) / (ACC_SENS * ACC_AMP);

	// ���̌X���̃I�t�Z�b�g�p�x�v�Z[rad]
	ad->tilt_angle_offset_rad[pitch] = asin(acc_g[pitch]);
	ad->tilt_angle_offset_rad[roll] = -asin(acc_g[roll]);

	// [rad] �� [deg]
	ad->tilt_angle_offset_deg[pitch] = ad->tilt_angle_offset_rad[pitch] * radtodeg;
	ad->tilt_angle_offset_deg[roll] = ad->tilt_angle_offset_rad[roll] * radtodeg;
	// ----------------------------------------------------------------------------------------------
}

//--------------------------------------//
//	FlexiForce(�ڒn����p)		//
//		FF_get()		//
//--------------------------------------//
void all_FF_get(ADType *ad, double ad_data[])
{
	int i;

	// �d���l�擾�i 0 �` 10.24[V] �j
	for(i=0; i<UNITS; i++) {
		ad->FF_data[i] = (float)ad_data[AD_FF_CH[i]] * -1.0;	// ���̒l�Ō��o�����̂ŁC-1�������Đ��̒l��

//		// �ڒn����
//		if(load->FF_data[i] >= 4.0)	//(load->FF_data_offset[i] + 1.0))	//�����l+1.0�ȏ�̂Ƃ�
//			load->touch[i] = 1;	// �ڒn
//		else
//			load->touch[i] = 0;	// ��ڒn
	}
}


//----------------------------------------------------------------------//
//	�|�e���V�����[�^(3382H-1-502)���瑫��v���[�g�̊p�x���擾	//
//				POT_get					//
//----------------------------------------------------------------------//
void all_POT_get(ADType *ad, double ad_data[], float h)
{
	int i;

	for(i=0; i<UNITS; i++) {
		// �|�e���V�����[�^�̓d��[V]���擾
		ad->POT_data[i] = (float)ad_data[AD_POT_CH[i]];

		// �d�����瑫��v���[�g�̕ψʊp�x���v�Z(���E�Ń|�e���V�����[�^�̌������t)
		// �o�͓d��[V] �~ ( �|�e���V�����[�^�̍ő匟�o�p�x(=330[deg]) / �ő�o�͓d��(=�����d��5[V]) )
		if(i == LF || i == LH) {
			ad->foot_angle[i] = ad->POT_data[i] * 330 * 0.2;
		}
		else {	// RF or RH
			ad->foot_angle[i] = 330 - (ad->POT_data[i] * 330 * 0.2);
		}

		// �I�t�Z�b�g�p�x����̑��Ίp�x���v�Z
		ad->foot_rel_angle[i]  = ad->foot_angle[i] - ad->foot_angle_offset[i]; // �I�t�Z�b�g�p�x����̑��Ίp�x

		// �p���x�̌v�Z
		ad->foot_angle_vel[i] = ( ad->foot_angle[i] - ad->foot_angle_last[i] ) / h;

		// ��O�̊p�x���X�V
		ad->foot_angle_last[i] = ad->foot_angle[i];
	}
}

//----------------------//
//	�ڒn���擾	//
//	touch_get()	//
//----------------------//
void touch_get(ADType *ad, LOADType *load)
{
	int 	i;	// �J�E���^

	// FlexiForce�ɂ��ڒn����
	// LF
	if(ad->FF_data[0] >= /*ad->FF_data_offset[0] +*/ FF_touch_th)	//(load->data_FF_last[i] + 1.0))	//�����l+1.0�ȏ�̂Ƃ�
		load->FF_touch[0] = 4;	// �ڒn
	else
		load->FF_touch[0] = 0;	// ��ڒn
	// LH
	if(ad->FF_data[1] >= /*ad->FF_data_offset[1] +*/ FF_touch_th)	//(load->data_FF_last[i] + 1.0))	//�����l+1.0�ȏ�̂Ƃ�
		load->FF_touch[1] = 3;	// �ڒn
	else
		load->FF_touch[1] = 0;	// ��ڒn
	// RF
	if(ad->FF_data[2] >= /*ad->FF_data_offset[2] +*/ FF_touch_th)	//(load->data_FF_last[i] + 1.0))	//�����l+1.0�ȏ�̂Ƃ�
		load->FF_touch[2] = 2;	// �ڒn
	else
		load->FF_touch[2] = 0;	// ��ڒn
	// RH
	if(ad->FF_data[3] >= /*ad->FF_data_offset[3] +*/ FF_touch_th)	//(load->data_FF_last[i] + 1.0))	//�����l+1.0�ȏ�̂Ƃ�
		load->FF_touch[3] = 1;	// �ڒn
	else
		load->FF_touch[3] = 0;	// ��ڒn


	// �|�e���V�����[�^�ɂ��ڒn����(���Ίp�x���p�����x�����ɂق�0�̎��ɔ�ڒn�Ƃ���)
	// LF
	if(ad->foot_rel_angle[0] < POT_touch_th)
		load->POT_touch[0] = 0;	// ��ڒn
	else
		load->POT_touch[0] = 4;	// �ڒn
	// LH
	if(ad->foot_rel_angle[1] < POT_touch_th)
		load->POT_touch[1] = 0;	// ��ڒn
	else
		load->POT_touch[1] = 3;	// �ڒn
	// RF
	if(ad->foot_rel_angle[2] < POT_touch_th)
		load->POT_touch[2] = 0;	// ��ڒn
	else
		load->POT_touch[2] = 2;	// �ڒn
	// RH
	if(ad->foot_rel_angle[3] < POT_touch_th)
		load->POT_touch[3] = 0;	// ��ڒn
	else
		load->POT_touch[3] = 1;	// �ڒn

	// �ŏI�I�ɍ̗p����ڒn���
	for(i=0; i<UNITS; i++) {
		load->touch[i] = load->FF_touch[i];
		//load->touch[i] = load->POT_touch[i];
	}
}

//----------------------------------------------------------------------//
//	�W���C���Z���T(ENC-03R)���瓷�̌X���p�x(Pitch, Roll)���擾	//
//				gyro_get()				//
//----------------------------------------------------------------------//
void gyro_get(ADType *ad, double ad_data[], float h)
{
	// �ϐ��錾
	float gyro_data_deviation[num_AXIS];	// �I�t�Z�b�g�o�͂���Ƃ����o�͕΍�
	float gyro_angle_variation[num_AXIS];	// �ЂƂO�̊p�x����̊p�x�ω���

	//�W���C���Z���T�̏o��[V]�擾
	ad->gyro_data[pitch] = (float)ad_data[AD_GYRO_CH[pitch]];
	ad->gyro_data[roll] = (float)ad_data[AD_GYRO_CH[roll]];

	// �I�t�Z�b�g�o��[V]����Ƃ����o�͕΍�[V]
	gyro_data_deviation[pitch] = ad->gyro_data[pitch] - ad->gyro_data_offset[pitch];
	gyro_data_deviation[roll] = ad->gyro_data[roll] - ad->gyro_data_offset[roll];

	// [V] �� [mV]
	gyro_data_deviation[pitch] *= 1000.0;
	gyro_data_deviation[roll] *= 1000.0;

	// �p���x[deg/sec]�̌v�Z(= �o�͕΍�[mV] / ������̃Z���T���x[mV/deg/sec])
	ad->gyro_angle_vel[pitch] = gyro_data_deviation[pitch] / GYRO_SENS_AMP;
	ad->gyro_angle_vel[roll] = gyro_data_deviation[roll] / GYRO_SENS_AMP;

	// �ЂƂO����̊p�x�ω���[deg]�̌v�Z
	gyro_angle_variation[pitch] = ad->gyro_angle_vel[pitch] * h;
	gyro_angle_variation[roll] = ad->gyro_angle_vel[roll] * h;

	// ���݂̊p�x�X�V[deg]
	ad->gyro_angle_deg[pitch] += gyro_angle_variation[pitch];	//�O����
	ad->gyro_angle_deg[roll] -= gyro_angle_variation[roll];		//�����C��,�E����

	// [deg] �� [rad]
	ad->gyro_angle_rad[pitch] = ad->gyro_angle_deg[pitch] * degtorad;
	ad->gyro_angle_rad[roll] = ad->gyro_angle_deg[roll] * degtorad;

	// ���[�p�X�ɂ�镽����[rad]
	ad->gyro_angle_lowp_rad[pitch] = ((ad->gyro_angle_lowp_rad_last[pitch] * 399.0) + ad->gyro_angle_rad[pitch]) / 400.0;
	ad->gyro_angle_lowp_rad[roll] = ((ad->gyro_angle_lowp_rad_last[roll] * 399.0) + ad->gyro_angle_rad[roll]) / 400.0;

	// �ЂƂO�̃��[�p�X�ʂ�����̊p�x�X�V
	ad->gyro_angle_lowp_rad_last[pitch] = ad->gyro_angle_lowp_rad[pitch];
	ad->gyro_angle_lowp_rad_last[roll] = ad->gyro_angle_lowp_rad[roll];

	// [rad] �� [deg]
	ad->gyro_angle_lowp_deg[pitch] = ad->gyro_angle_lowp_rad[pitch] * radtodeg;
	ad->gyro_angle_lowp_deg[roll] = ad->gyro_angle_lowp_rad[roll] * radtodeg;
}


//------------------------------------------------------------------------------//
//	�����x�Z���T(Crossbow CXL02LF3)���瓷�̌X���p�x(Pitch, Roll)���擾	//
//				acc_get()					//
//------------------------------------------------------------------------------//
void acc_get(ADType *ad, double ad_data[], float h)
{
	// �ϐ��錾
	float acc[num_AXIS];		//�����x����p�x�ւ̌v�Z
	float acc_nor[num_AXIS];	//���K��

	//�����x�Z���T�̏o��[V]�擾
	ad->acc_data[pitch] = (float)ad_data[AD_ACC_CH[pitch]];
	ad->acc_data[roll] = (float)ad_data[AD_ACC_CH[roll]];

	// �p�x�ϊ���
	acc[pitch] = (ad->acc_data[pitch] - (2.5 * ACC_AMP)) / (2.0 * ACC_AMP);
	acc[roll] = (ad->acc_data[roll] - (2.5 * ACC_AMP)) / (2.0 * ACC_AMP);

	//���K��
	acc_nor[pitch] = acc[pitch] / 1.25;
	acc_nor[roll] = acc[roll] / 1.25;
	if(acc_nor[pitch] > 1.0){
		acc_nor[pitch] = 1.0;
	}
	if(acc_nor[pitch] < -1.0){
		acc_nor[pitch] = -1.0;
	}

	if(acc_nor[roll] > 1.0){
		acc_nor[roll] = 1.0;
	}
	if(acc_nor[roll] < -1.0){
		acc_nor[roll] = -1.0;
	}

	//�p�x�v�Z
	//[rad]
	ad->acc_angle_rad[pitch] = asin(acc_nor[pitch]);
	ad->acc_angle_rad[roll] = -asin(acc_nor[roll]);
	//[deg]
	ad->acc_angle_deg[pitch] = ad->acc_angle_rad[pitch] * radtodeg;
	ad->acc_angle_deg[roll] = ad->acc_angle_rad[roll] * radtodeg;
	//inc->acc_inc_deg[pitch] = inc->acc_data[pitch];
	//inc->acc_inc_deg[roll] = inc->acc_data[roll];

	// ���[�p�X�t�B���^�ɂ�镽����[rad]
	ad->acc_angle_lowp_rad[pitch] = (ad->acc_angle_lowp_rad_last[pitch] * 199.0 + ad->acc_angle_rad[pitch]) / 200.0;
	ad->acc_angle_lowp_rad[roll] = (ad->acc_angle_lowp_rad_last[roll] * 199.0 + ad->acc_angle_rad[roll]) / 200.0;

	// �ЂƂO�̃��[�p�X�ʂ�����̊p�x�X�V
	ad->acc_angle_lowp_rad_last[pitch] = ad->acc_angle_lowp_rad[pitch];	//�X�V
	ad->acc_angle_lowp_rad_last[roll] = ad->acc_angle_lowp_rad[roll];

	// [rad] �� [deg]
	ad->acc_angle_lowp_deg[pitch] = ad->acc_angle_lowp_rad[pitch] * radtodeg;
	ad->acc_angle_lowp_deg[roll] = ad->acc_angle_lowp_rad[roll] * radtodeg;
}



//------------------------------//
//	���̌X���p�x�̌���	//
//	    tilt_con()		//
//------------------------------//
void tilt_con(ADType *ad)
{
	// �ϐ��錾
	float drift[num_AXIS];	//�h���t�g�l

	// �h���t�g�l���v�Z
	drift[pitch] = ad->gyro_angle_lowp_rad[pitch] - ad->acc_angle_lowp_rad[pitch];
	drift[roll] = ad->gyro_angle_lowp_rad[roll] - ad->acc_angle_lowp_rad[roll];

	// ���̌X�Ίp�x�̌v�Z[rad]
	ad->tilt_angle_rad[pitch] = ad->gyro_angle_rad[pitch] - drift[pitch];
	ad->tilt_angle_rad[roll] = ad->gyro_angle_rad[roll] - drift[roll];

	// [rad] �� [deg]
	ad->tilt_angle_deg[pitch] = ad->tilt_angle_rad[pitch] * radtodeg;
	ad->tilt_angle_deg[roll] = ad->tilt_angle_rad[roll] * radtodeg;
}


//----------------------//
//	��C���擾	//
//	air_press_get()	//
//----------------------//
void air_press_get(ADType *ad, double ad_data[])
{

	// �C���Z���T�l[V]���擾
	ad->air_press_data = (float)(ad_data[AD_AIR_PRESS_CH]);

	// �C��[MPa]�֕ϊ�
	// �ϊ����͋�C���Z���T(PSE530-M5-L)�̐����p�A�i���O�o�͂̓����O���t��蓱�o
	ad->atmosphere = 0.25 * ad->air_press_data - 0.25;
}