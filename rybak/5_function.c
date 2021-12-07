// -------------------------------------------- //
//	locomotion�Ɋ֗^���Ă��Ȃ��֐��Q	//
//	function.h				//
// -------------------------------------------- //

#include "8_define.h"
#include "9_user_param.h"


/*--------------------------------------*/
/*	�e�T���v���f�[�^��z��Ɋi�[	*/
/*	data_get()			*/
/*--------------------------------------*/
void data_get(int k, float t, SAMType *sam, RybakUnit Unit[], ENCType *enc, ADType *ad, LOADType *load)
{
	/*------�ϐ��錾------*/
	int i;	//�J�E���^


	//���Ԃ̕ۑ�
	sam->t[k] = t - Time_Step4;	// ���s�J�n������0[s]�Ƃ���

	for(i=0; i<UNITS; i++){
		//CPG�����̕ۑ�
		sam->cpg_cycle[i][k] = Unit[i].cpg_cycle;
		//RG�j���[�����̖��d�ʂ̕ۑ�
		sam->RG_F_V[i][k] = Unit[i].Neuron_V[RG_F];
		sam->RG_E_V[i][k] = Unit[i].Neuron_V[RG_E];
		//PF�j���[�����̖��d�ʂ̕ۑ�
		sam->PF_Sw_V[i][k] = Unit[i].Neuron_V[PF_Sw];
		sam->PF_St_V[i][k] = Unit[i].Neuron_V[PF_St];
		sam->PF_Lo_V[i][k] = Unit[i].Neuron_V[PF_Lo];
		sam->PF_Td_V[i][k] = Unit[i].Neuron_V[PF_Td];
		// �^���j���[�����̖��d�ʂ̕ۑ�
		sam->Mn_1_V[i][k] = Unit[i].Neuron_V[Mn_1];
		sam->Mn_2_V[i][k] = Unit[i].Neuron_V[Mn_2];
		sam->Mn_3_V[i][k] = Unit[i].Neuron_V[Mn_3];
		sam->Mn_4_V[i][k] = Unit[i].Neuron_V[Mn_4];
		sam->Mn_5_V[i][k] = Unit[i].Neuron_V[Mn_5];
		sam->Mn_6_V[i][k] = Unit[i].Neuron_V[Mn_6];
        //�r���׏��̕ۑ� �V�O���C�h��
//		sam->feed_prs[i][k] = 1.0 / ( 1.0 + exp( -(load->load_value[i] - 3.0) * sigmoid_alpha) );
		//�r���׏��̕ۑ� �V�O���C�h�O
		sam->feed_prs[i][k] =load->load_value[i];
		//RG-F�̔��Ί��Ԃ̕ۑ�
		sam->RG_F_period[i][k] = Unit[i].RG_F_period;
		//RG-E�̔��Ί��Ԃ̕ۑ�
		sam->RG_E_period[i][k] = Unit[i].RG_E_period;
		//���֐ߊp�x�̕ۑ�
		sam->angle_hip[i][k] = enc->angle_hip_deg[i];
		//�G�֐ߊp�x�̕ۑ�
		sam->angle_knee[i][k] = enc->angle_knee_deg[i];
		//����֐ߊp�x�̕ۑ�
		sam->angle_ankle[i][k] = enc->angle_ankle_deg[i];
		//FF�d���l�̕ۑ�
		sam->FF_data[i][k] = ad->FF_data[i];
		//�ڒn���(FF�ˑ�)
		sam->FF_touch[i][k] = load->FF_touch[i];
		//�|�e���V�����[�^�d���l�̕ۑ�
		sam->POT_data[i][k] = ad->POT_data[i];
		//����v���[�g�p�x�̕ۑ�
		sam->foot_angle[i][k] = ad->foot_angle[i];
		//����v���[�g�p���x�̕ۑ�
		sam->foot_angle_vel[i][k] = ad->foot_angle_vel[i];
		//�ڒn���(�|�e���V�����[�^�ˑ�)
		sam->POT_touch[i][k] = load->POT_touch[i];
		//�ڒn���(�����)�̕ۑ�
		sam->touch[i][k] = load->touch[i];
		//20191126�ǉ��@�r���חp��݃j���[����In_Feed�̕ۑ�
		sam->In_Feed_V[i][k] = Unit[i].Neuron_V[In_Feed];
        //�ؓ����샂�[�h�̕ۑ�
        sam->Valve_mode1[i][k] = valve_mode[i][0];
        sam->Valve_mode2[i][k] = valve_mode[i][1];
        sam->Valve_mode3[i][k] = valve_mode[i][2];
        sam->Valve_mode4[i][k] = valve_mode[i][3];
        sam->Valve_mode5[i][k] = valve_mode[i][4];
        sam->Valve_mode6[i][k] = valve_mode[i][5];
	}

	for(i=0; i<num_AXIS; i++) {
		// �W���C���Z���T�o��[V]
		sam->gyro_data[i][k] = ad->gyro_data[i];
		// �W���C���Z���T�ɂ�茟�o�������̌X��[deg]
		sam->gyro_angle_deg[i][k] = ad->gyro_angle_deg[i] + ad->tilt_angle_offset_deg[i];
		// �W���C���Z���T�ɂ�茟�o�������̌X��[deg](���[�p�X�ʉߌ�)
		sam->gyro_angle_lowp_deg[i][k] = ad->gyro_angle_lowp_deg[i] + ad->tilt_angle_offset_deg[i];
		// �����x�Z���T�o��[V]
		sam->acc_data[i][k] = ad->acc_data[i];
		// �����x�Z���T�ɂ�茟�o�������̌X��[deg]
		sam->acc_angle_deg[i][k] = ad->acc_angle_deg[i];
		// �����x�Z���T�ɂ�茟�o�������̌X��[deg](���[�p�X�ʉߌ�)
		sam->acc_angle_lowp_deg[i][k] = ad->acc_angle_lowp_deg[i];
		// �W���C���Z���T�Ɖ����x�Z���T�𕹗p���ăh���t�g�␳�������̌X���p�x[deg]
		sam->tilt_angle_deg[i][k] = ad->tilt_angle_deg[i] + ad->tilt_angle_offset_deg[i];
	}

//printf(" %f", sam->gyro_angle_deg[pitch][k]);
//printf(" %f\n", ad->gyro_angle_deg[pitch]);

	//�g���b�h�~�����x�̕ۑ�
	sam->speed_treadmill[k] = enc->speed_treadmill;

	//�l�H�ؓ����C���̎擾
	sam->atmosphere[k] = ad->atmosphere;

    //�d���ق�ONOFF���擾
    sam->muscle_io[k] = muscle_io;

}


/*----------------------------------------------*/
/*	�t�@�C������p�����[�^��ǂݍ���	*/
/*	file_output()				*/
/*----------------------------------------------*/
int file_input(float buf[30][30], float c_f[][DEF_B], float a_f[][DEF_B], float b_f[][DEF_B], float w_f[][DEF_B], float c_h[][DEF_B], float a_h[][DEF_B], float b_h[][DEF_B], float w_h[][DEF_B])
{
	//------�ϐ��錾------//
	int i, j;	// �J�E���^
	FILE	*fpin_param = fopen("0_param.txt", "r");		//�`���[�j���O�p���̓t�@�C�����J��

	//�t�@�C���W�J���̈��S���u
	if (fpin_param == NULL) {
		printf("�t�@�C�����g�p���ł��D\n");
		return 1;
	}

	// MAT���̃p�����[�^(PF->Mn�ȊO)���� -------------------------------------------------------------------------------------
	//	PF->Mn�ȊO�̌����׏d��4�r�œ���
	//20191126 In_Feed�p��buf[i][24]�ǉ�
	// MATc_fore/hind
	for(i=0; i<MAT_C_INPUTS; i++){
		fscanf(fpin_param, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
					&buf[i][0], &buf[i][1], &buf[i][2], &buf[i][3], &buf[i][4], &buf[i][5],
					&buf[i][6], &buf[i][7], &buf[i][8], &buf[i][9], &buf[i][10], &buf[i][11],
					&buf[i][12], &buf[i][13], &buf[i][14], &buf[i][15], &buf[i][16], &buf[i][17],
					&buf[i][18], &buf[i][19], &buf[i][20], &buf[i][21], &buf[i][22], &buf[i][23], &buf[i][24]	);
		for(j=0; j<DEF_B; j++)
			c_f[i][j] = c_h[i][j] = buf[i][j];
	}
	// MATa_fore/hind
	for(i=0; i<MAT_A_INPUTS; i++){
		fscanf(fpin_param, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
					&buf[i][0], &buf[i][1], &buf[i][2], &buf[i][3], &buf[i][4], &buf[i][5],
					&buf[i][6], &buf[i][7], &buf[i][8], &buf[i][9], &buf[i][10], &buf[i][11],
					&buf[i][12], &buf[i][13], &buf[i][14], &buf[i][15], &buf[i][16], &buf[i][17],
					&buf[i][18], &buf[i][19], &buf[i][20], &buf[i][21], &buf[i][22], &buf[i][23], &buf[i][24]	);
		for(j=0; j<DEF_B; j++)
			a_f[i][j] = a_h[i][j] = buf[i][j];
	}
	// MATb_fore/hind
	for(i=0; i<MAT_B_INPUTS; i++){
		fscanf(fpin_param, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
					&buf[i][0], &buf[i][1], &buf[i][2], &buf[i][3], &buf[i][4], &buf[i][5],
					&buf[i][6], &buf[i][7], &buf[i][8], &buf[i][9], &buf[i][10], &buf[i][11],
					&buf[i][12], &buf[i][13], &buf[i][14], &buf[i][15], &buf[i][16], &buf[i][17],
					&buf[i][18], &buf[i][19], &buf[i][20], &buf[i][21], &buf[i][22], &buf[i][23], &buf[i][24]	);
		for(j=0; j<DEF_B; j++)
			b_f[i][j] = b_h[i][j] = buf[i][j];
	}
	// MATw_fore/hind
	for(i=0; i<MAT_W_INPUTS; i++){
		fscanf(fpin_param, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
					&buf[i][0], &buf[i][1], &buf[i][2], &buf[i][3], &buf[i][4], &buf[i][5],
					&buf[i][6], &buf[i][7], &buf[i][8], &buf[i][9], &buf[i][10], &buf[i][11],
					&buf[i][12], &buf[i][13], &buf[i][14], &buf[i][15], &buf[i][16], &buf[i][17],
					&buf[i][18], &buf[i][19], &buf[i][20], &buf[i][21], &buf[i][22], &buf[i][23], &buf[i][24]	);
		for(j=0; j<DEF_B; j++)
			w_f[i][j] = w_h[i][j] = buf[i][j];
	}
	// -----------------------------------------------------------------------------------------------------------------------
	// PF->Mn�̌����׏d���� --------------------------------------------------------------------------------------------------
	//	PF->Mn�̌����׏d�͑O��r�ňقȂ�
	for(i=0; i<8; i++){
		fscanf(fpin_param, "%f %f %f %f %f %f", &buf[i][0], &buf[i][1], &buf[i][2], &buf[i][3], &buf[i][4], &buf[i][5]);
	}
	for(j=0; j<MUSCLES_PER_LEG; j++)	{
		// �O�r
		a_f[2][Mn_1+j] = buf[0][j];
		a_f[3][Mn_1+j] = buf[1][j];
		a_f[5][Mn_1+j] = buf[2][j];
		a_f[6][Mn_1+j] = buf[3][j];
		// ��r
		a_h[2][Mn_1+j] = buf[4][j];
		a_h[3][Mn_1+j] = buf[5][j];
		a_h[5][Mn_1+j] = buf[6][j];
		a_h[6][Mn_1+j] = buf[7][j];
	}
	// -----------------------------------------------------------------------------------------------------------------------

	// �ǂݍ��݃t�@�C���� ����
	fclose(fpin_param);
	return 0;
}

/*----------------------------------------------*/
/*	�t�@�C���փT���v���f�[�^�������o��	*/
/*	file_output()				*/
/*----------------------------------------------*/
int file_output(int k, SAMType *sam)
{
	/*------�ϐ��錾------*/
	int	i, sam_n;	//�J�E���^
	float	cpg_phase[UNITS];
	float	RG_F_fV[UNITS];
	float	RG_E_fV[UNITS];
	float	PF_Sw_fV[UNITS];
	float	PF_St_fV[UNITS];
	float	PF_Lo_fV[UNITS];
	float	PF_Td_fV[UNITS];
	float	Mn_1_fV[UNITS];
	float	Mn_2_fV[UNITS];
	float	Mn_3_fV[UNITS];
	float	Mn_4_fV[UNITS];
	float	Mn_5_fV[UNITS];
	float	Mn_6_fV[UNITS];
    int	Mn_1_ONOFF[UNITS];
	int	Mn_2_ONOFF[UNITS];
	int	Mn_3_ONOFF[UNITS];
	int	Mn_4_ONOFF[UNITS];
	int	Mn_5_ONOFF[UNITS];
	int	Mn_6_ONOFF[UNITS];
	float	Muscle6_len[UNITS];
	float	duty_ratio[UNITS];
	float In_Feed_fV[UNITS];

	//�擾�f�[�^�ۑ��t�@�C��
	FILE	*fp_neuron = fopen("0_log_neuron.txt", "w");	//CPG���̊e�j���[�����̔��Η���ۑ�
	FILE	*fp_sensor = fopen("0_log_sensor.txt", "w");	//�r�̊֐ߊp�x�⓷�̌X���i�\��j�Ȃǂ�ۑ�


	//�t�@�C���W�J���̈��S���u
	if (fp_neuron == NULL) {
		printf("�t�@�C�����g�p���ł��Dneuron\n");
		return 1;
	}
	if (fp_sensor == NULL) {
		printf("�t�@�C�����g�p���ł��Dsensor\n");
		return 1;
	}

	/*------�j���[�����f�[�^�����o��------*/
	//�ۑ�����f�[�^���ڂ̗�
	fprintf(fp_neuron, "t \t vel_treadmill \t ");	//�����A�g���b�h�~�����x
	fprintf(fp_neuron, "CPG_LF \t CPG_LH \t CPG_RF \t CPG_RH \t ");	//CPG�g�`
	fprintf(fp_neuron, "Cycle_LF \t Cycle_LH \t Cycle_RF \t Cycle_RH \t ");	// CPG����

	fprintf(fp_neuron, "RG-F_fV_LF \t RG-E_fV_LF \t ");	//RG�j���[�����̔��Η��i���O�j
	fprintf(fp_neuron, "RG-F_fV_LH \t RG-E_fV_LH \t ");	//RG�j���[�����̔��Η��i����j
	fprintf(fp_neuron, "RG-F_fV_RF \t RG-E_fV_RF \t ");	//RG�j���[�����̔��Η��i�E�O�j
	fprintf(fp_neuron, "RG-F_fV_RH \t RG-E_fV_RH \t ");	//RG�j���[�����̔��Η��i�E��j

	fprintf(fp_neuron, "RG-F_V_LF \t RG-E_V_LF \t ");	//RG�j���[�����̖��d�ʁi���O�j
	fprintf(fp_neuron, "RG-F_V_LH \t RG-E_V_LH \t ");	//RG�j���[�����̖��d�ʁi����j
	fprintf(fp_neuron, "RG-F_V_RF \t RG-E_V_RF \t ");	//RG�j���[�����̖��d�ʁi�E�O�j
	fprintf(fp_neuron, "RG-F_V_RH \t RG-E_V_RH \t ");	//RG�j���[�����̖��d�ʁi�E��j

	fprintf(fp_neuron, "PF-Sw_LF \t PF-St_LF \t PF-Lo_LF \t PF-Td_LF \t ");	//PF�j���[�����̔��Η��i���O�j
	fprintf(fp_neuron, "PF-Sw_LH \t PF-St_LH \t PF-Lo_LH \t PF-Td_LH \t ");	//PF�j���[�����̔��Η��i����j
	fprintf(fp_neuron, "PF-Sw_RF \t PF-St_RF \t PF-Lo_RF \t PF-Td_RF \t ");	//PF�j���[�����̔��Η��i�E�O�j
	fprintf(fp_neuron, "PF-Sw_RH \t PF-St_RH \t PF-Lo_RH \t PF-Td_RH \t ");	//PF�j���[�����̔��Η��i�E��j

	fprintf(fp_neuron, "Mn-1_LF \t Mn-2_LF \t Mn-3_LF \t Mn-4_LF \t Mn-5_LF \t Mn-6_LF \t ");	//�^���j���[�����̔��Η��i���O�j
	fprintf(fp_neuron, "Mn-1_LH \t Mn-2_LH \t Mn-3_LH \t Mn-4_LH \t Mn-5_LH \t Mn-6_LH \t ");	//�^���j���[�����̔��Η��i����j
	fprintf(fp_neuron, "Mn-1_RF \t Mn-2_RF \t Mn-3_RF \t Mn-4_RF \t Mn-5_RF \t Mn-6_RF \t ");	//�^���j���[�����̔��Η��i�E�O�j
	fprintf(fp_neuron, "Mn-1_RH \t Mn-2_RH \t Mn-3_RH \t Mn-4_RH \t Mn-5_RH \t Mn-6_RH \t ");	//�^���j���[�����̔��Η��i�E��j

	fprintf(fp_neuron, "Mn-1_LF_ONOFF \t Mn-2_LF_ONOFF \t Mn-3_LF_ONOFF \t Mn-4_LF_ONOFF \t Mn-5_LF_ONOFF \t Mn-6_LF_ONOFF \t ");	//�^���j���[������ONOFF�i���O�j
	fprintf(fp_neuron, "Mn-1_LH_ONOFF \t Mn-2_LH_ONOFF \t Mn-3_LH_ONOFF \t Mn-4_LH_ONOFF \t Mn-5_LH_ONOFF \t Mn-6_LH_ONOFF \t ");	//�^���j���[������ONOFF�i����j
	fprintf(fp_neuron, "Mn-1_RF_ONOFF \t Mn-2_RF_ONOFF \t Mn-3_RF_ONOFF \t Mn-4_RF_ONOFF \t Mn-5_RF_ONOFF \t Mn-6_RF_ONOFF \t ");	//�^���j���[������ONOFF�i�E�O�j
	fprintf(fp_neuron, "Mn-1_RH_ONOFF \t Mn-2_RH_ONOFF \t Mn-3_RH_ONOFF \t Mn-4_RH_ONOFF \t Mn-5_RH_ONOFF \t Mn-6_RH_ONOFF \t ");	//�^���j���[������ONOFF�i�E��j

	fprintf(fp_neuron, "touch_LF \t touch_LH \t touch_RF \t touch_RH \t ");	//�ڒn���

	fprintf(fp_neuron, "feed_prs_LF \t feed_prs_LH \t feed_prs_RF \t feed_prs_RH \t ");	//�r���׏��

	fprintf(fp_neuron, "RG-F_period_LF \t RG-E_period_LF \t RG-F_period_LH \t RG-E_period_LH \t RG-F_period_RF \t RG-E_period_RF \t RG-F_period_RH \t RG-E_period_RH \t ");	// RG�̔��Ί���

	fprintf(fp_neuron, "duty_ratio_LF \t duty_ratio_LH \t duty_ratio_RF \t duty_ratio_RH \t "); //duty��

	fprintf(fp_neuron, "In_Feed_LF \t In_Feed_LH \t In_Feed_RF \t In_Feed_RH \t "); //20191126�@�r���ׂ�ʂ���݃j���[�����̔��Η�

	//fprintf(fp_neuron, "In_F_LF \t In_E_LF \t In_LF \t Inab_E_LF \t In_PF-Sw_LF \t In_PF-St_LF \t In_PF-Lo_LF \t In_PF-Td_LF \t");	//��݃j���[�����i���O�j

    fprintf(fp_neuron,"muscle_io \t ");

	fprintf(fp_neuron, "\n");	// ���s

	//�ۑ��f�[�^�̏����o��
	for(sam_n=0; sam_n<k; sam_n++){
		fprintf(fp_neuron, "%f \t %f \t ", sam->t[sam_n], sam->speed_treadmill[sam_n]);

		// CPG�g�`�E����
		for (i=0; i<UNITS; i++)
			cpg_phase[i] = GetfV(sam->RG_E_V[i][sam_n], RG_E) - GetfV(sam->RG_F_V[i][sam_n], RG_F);
		fprintf(fp_neuron, "%f \t %f \t %f \t %f \t ", cpg_phase[0], cpg_phase[1], cpg_phase[2], cpg_phase[3]);
		fprintf(fp_neuron, "%f \t %f \t %f \t %f \t ", sam->cpg_cycle[0][sam_n], sam->cpg_cycle[1][sam_n], sam->cpg_cycle[2][sam_n], sam->cpg_cycle[3][sam_n]);

		//RG�j���[�����̔��Η�
		for(i=0; i<UNITS; i++) {
			RG_F_fV[i] = GetfV(sam->RG_F_V[i][sam_n], RG_F);
			RG_E_fV[i] = GetfV(sam->RG_E_V[i][sam_n], RG_E);
			fprintf(fp_neuron, "%f \t %f \t ", RG_F_fV[i], RG_E_fV[i]);
		}
		//RG�j���[�����̖��d��
		for(i=0; i<UNITS; i++)
			fprintf(fp_neuron, "%f \t %f \t ", sam->RG_F_V[i][sam_n], sam->RG_E_V[i][sam_n]);

		//PF�j���[�����̔��Η�
		for(i=0; i<UNITS; i++) {
			PF_Sw_fV[i] = GetfV(sam->PF_Sw_V[i][sam_n], PF_Sw);
			PF_St_fV[i] = GetfV(sam->PF_St_V[i][sam_n], PF_St);
			PF_Lo_fV[i] = GetfV(sam->PF_Lo_V[i][sam_n], PF_Lo);
			PF_Td_fV[i] = GetfV(sam->PF_Td_V[i][sam_n], PF_Td);
			fprintf(fp_neuron, "%f \t %f \t %f \t %f \t ", PF_Sw_fV[i], PF_St_fV[i], PF_Lo_fV[i], PF_Td_fV[i]);
		}
		//Mn�j���[�����̔��Η�
		for(i=0; i<UNITS; i++) {
			Mn_1_fV[i] = GetfV(sam->Mn_1_V[i][sam_n], Mn_1);
			Mn_2_fV[i] = GetfV(sam->Mn_2_V[i][sam_n], Mn_2);
			Mn_3_fV[i] = GetfV(sam->Mn_3_V[i][sam_n], Mn_3);
			Mn_4_fV[i] = GetfV(sam->Mn_4_V[i][sam_n], Mn_4);
			Mn_5_fV[i] = GetfV(sam->Mn_5_V[i][sam_n], Mn_5);
			Mn_6_fV[i] = GetfV(sam->Mn_6_V[i][sam_n], Mn_6);
			fprintf(fp_neuron, "%f \t %f \t %f \t %f \t %f \t %f \t ", Mn_1_fV[i], Mn_2_fV[i], Mn_3_fV[i], Mn_4_fV[i], Mn_5_fV[i], Mn_6_fV[i]);
		}

		//Mn�j���[������ONOFF
		for(i=0; i<UNITS; i++) {
			Mn_1_ONOFF[i] = sam->Valve_mode1[i][sam_n];
            Mn_2_ONOFF[i] = sam->Valve_mode2[i][sam_n];
            Mn_3_ONOFF[i] = sam->Valve_mode3[i][sam_n];
            Mn_4_ONOFF[i] = sam->Valve_mode4[i][sam_n];
            Mn_5_ONOFF[i] = sam->Valve_mode5[i][sam_n];
            Mn_6_ONOFF[i] = sam->Valve_mode6[i][sam_n];

            fprintf(fp_neuron, "%d \t %d \t %d \t %d \t %d \t %d \t ", Mn_1_ONOFF[i], Mn_2_ONOFF[i], Mn_3_ONOFF[i], Mn_4_ONOFF[i], Mn_5_ONOFF[i], Mn_6_ONOFF[i]);

		}

		//�ڒn���
		fprintf(fp_neuron, "%d \t %d \t %d \t %d \t ", sam->touch[0][sam_n], sam->touch[1][sam_n], sam->touch[2][sam_n], sam->touch[3][sam_n]);

		//�r���׏��
		for(i=0; i<UNITS; i++)
			fprintf(fp_neuron, "%f \t ", sam->feed_prs[i][sam_n]);

		//RG�̔��Ί���
		for(i=0; i<UNITS; i++)
			fprintf(fp_neuron, "%f \t %f \t ", sam->RG_F_period[i][sam_n], sam->RG_E_period[i][sam_n]);

		//duty��
		for(i=0; i<UNITS; i++) {
			duty_ratio[i] = sam->RG_E_period[i][sam_n] / (sam->RG_F_period[i][sam_n] + sam->RG_E_period[i][sam_n]);
			fprintf(fp_neuron, "%f \t ", duty_ratio[i]);
		}

		for(i=0; i<UNITS; i++) {
				In_Feed_fV[i] = GetfV(sam->In_Feed_V[i][sam_n], In_Feed);
				fprintf(fp_neuron, "%f \t ", In_Feed_fV[i]);
		}

        fprintf(fp_neuron, "%d \t ", sam->muscle_io[sam_n]);

		fprintf(fp_neuron, "\n");	// ���s
	}

	/*------�Z���T�f�[�^�̏����o��------*/
	//�ۑ�����f�[�^���ڂ̗�
	fprintf(fp_sensor, "t \t vel_treadmill \t");	//�����C�g���b�h�~�����x

	fprintf(fp_sensor, "CPG_LF \t CPG_LH \t CPG_RF \t CPG_RH \t");		// CPG�g�`
	fprintf(fp_sensor, "Cycle_LF \t Cycle_LH \t Cycle_RF \t Cycle_RH \t");	// CPG����

	fprintf(fp_sensor, "scapula_LF \t shoulder_LF \t elbow_LF \t ");	// ���O�r
	fprintf(fp_sensor, "hip_LH \t knee_LH \t ankle_LH \t ");		// ����r
	fprintf(fp_sensor, "scapula_RF \t shoulder_RF \t elbow_RF \t ");	// �E�O�r
	fprintf(fp_sensor, "hip_RH \t knee_RH \t ankle_RH \t ");		// �E��r


	fprintf(fp_sensor, "touch_LF \t touch_LH \t touch_RF \t touch_RH \t ");	// �ڒn���

	fprintf(fp_sensor, "Muscle6_len_LF \t Muscle6_len_LH \t Muscle6_len_RF \t Muscle6_len_RH \t ");	// 6�ԋؒ���

	fprintf(fp_sensor, "tilt_angle_pitch \t tilt_angle_roll \t ");			// ���̌X���p�x(�����)

	fprintf(fp_sensor, "FF_data_LF \t FF_data_LH \t FF_data_RF \t FF_data_RH \t ");		// FF�d���l
	fprintf(fp_sensor, "FF_touch_LF \t FF_touch_LH \t FF_touch_RF \t FF_touch_RH \t ");	// �ڒn���(FF�ˑ�)

	fprintf(fp_sensor, "POT_data_LF \t POT_data_LH \t POT_data_RF \t POT_data_RH \t "); 				// �|�e���V�����[�^�d���l
	fprintf(fp_sensor, "foot_angle_LF \t foot_angle_LH \t foot_angle_RF \t foot_angle_RH \t ");			// ����v���[�g�p�x
	fprintf(fp_sensor, "foot_angle_vel_LF \t foot_angle_vel_LH \t foot_angle_vel_RF \t foot_angle_vel_RH \t ");	// ����v���[�g�p���x
	fprintf(fp_sensor, "POT_touch_LF \t POT_touch_LH \t POT_touch_RF \t POT_touch_RH \t "); 			// �ڒn���(�|�e���V�����[�^�ˑ�)

	fprintf(fp_sensor, "gyro_data_pitch \t gyro_data_roll \t ");			// �W���C���Z���T�o��
	fprintf(fp_sensor, "gyro_angle_pitch \t gyro_angle_roll \t ");			// ���̌X���p�x(�W���C���Z���T)
	fprintf(fp_sensor, "gyro_angle_lowp_pitch \t gyro_angle_lowp_roll \t ");	// ���[�p�X�ʂ�����̓��̌X���p�x(�W���C���Z���T)

	fprintf(fp_sensor, "acc_data_pitch \t acc_data_roll \t ");			// �����x�Z���T�o��
	fprintf(fp_sensor, "acc_angle_pitch \t acc_angle_roll \t ");			// ���̌X���p�x(�����x�Z���T)
	fprintf(fp_sensor, "acc_angle_lowp_pitch \t acc_angle_lowp_roll \t ");		// ���[�p�X�ʂ�����̓��̌X���p�x(�����x�Z���T)

	fprintf(fp_sensor, "atmosphere \t ");	// �l�H�ؓ����C��

	fprintf(fp_sensor, "\n");	// ���s

	//�ۑ��f�[�^�̏����o��
	for(sam_n=0; sam_n<k; sam_n++){
		fprintf(fp_sensor, "%f \t %f \t ", sam->t[sam_n], sam->speed_treadmill[sam_n]);

		// CPG�g�`�E����
		for (i=0; i<UNITS; i++)
			cpg_phase[i] = GetfV(sam->RG_E_V[i][sam_n], RG_E) - GetfV(sam->RG_F_V[i][sam_n], RG_F);
		fprintf(fp_sensor, "%f \t %f \t %f \t %f \t ", cpg_phase[0], cpg_phase[1], cpg_phase[2], cpg_phase[3]);
		fprintf(fp_sensor, "%f \t %f \t %f \t %f \t ", sam->cpg_cycle[0][sam_n], sam->cpg_cycle[1][sam_n], sam->cpg_cycle[2][sam_n], sam->cpg_cycle[3][sam_n]);

		//�֐߃f�[�^
		for(i=0; i<UNITS; i++)
			fprintf(fp_sensor, "%f \t %f \t %f \t ", sam->angle_hip[i][sam_n], sam->angle_knee[i][sam_n], sam->angle_ankle[i][sam_n]);


		//�ڒn���
		fprintf(fp_sensor, "%d \t %d \t %d \t %d \t ", sam->touch[0][sam_n], sam->touch[1][sam_n], sam->touch[2][sam_n], sam->touch[3][sam_n]);

		// 6�ԋؒ���
		for (i=0; i<UNITS; i++) {
			if (i==LF || i==RF)	// �O�r
				Muscle6_len[i] = Calc_Muscle6_F_len(sam->angle_knee[i][sam_n], sam->angle_ankle[i][sam_n]);
			else	// ��r
				Muscle6_len[i] = Calc_Muscle6_H_len(sam->angle_knee[i][sam_n], sam->angle_ankle[i][sam_n]);
		}
		fprintf(fp_sensor, "%f \t %f \t %f \t %f \t ", Muscle6_len[0], Muscle6_len[1], Muscle6_len[2], Muscle6_len[3]);

		//���̌X���p�x(�����)
		fprintf(fp_sensor, "%f \t %f \t ", sam->tilt_angle_deg[0][sam_n], sam->tilt_angle_deg[1][sam_n]);

		//FF�d���l
		fprintf(fp_sensor, "%f \t %f \t %f \t %f \t ", sam->FF_data[0][sam_n], sam->FF_data[1][sam_n], sam->FF_data[2][sam_n], sam->FF_data[3][sam_n]);
		//�ڒn���(FF�ˑ�)
		fprintf(fp_sensor, "%d \t %d \t %d \t %d \t ", sam->FF_touch[0][sam_n], sam->FF_touch[1][sam_n], sam->FF_touch[2][sam_n], sam->FF_touch[3][sam_n]);

		//�|�e���V�����[�^�d���l
		fprintf(fp_sensor, "%f \t %f \t %f \t %f \t ", sam->POT_data[0][sam_n], sam->POT_data[1][sam_n], sam->POT_data[2][sam_n], sam->POT_data[3][sam_n]);
		//����v���[�g�p�x
		fprintf(fp_sensor, "%f \t %f \t %f \t %f \t ", sam->foot_angle[0][sam_n], sam->foot_angle[1][sam_n], sam->foot_angle[2][sam_n], sam->foot_angle[3][sam_n]);
		//����v���[�g�p���x
		fprintf(fp_sensor, "%f \t %f \t %f \t %f \t ", sam->foot_angle_vel[0][sam_n], sam->foot_angle_vel[1][sam_n], sam->foot_angle_vel[2][sam_n], sam->foot_angle_vel[3][sam_n]);
		//�ڒn���(�|�e���V�����[�^�ˑ�)
		fprintf(fp_sensor, "%d \t %d \t %d \t %d \t ", sam->POT_touch[0][sam_n], sam->POT_touch[1][sam_n], sam->POT_touch[2][sam_n], sam->POT_touch[3][sam_n]);

		//�W���C���Z���T�o��
		fprintf(fp_sensor, "%f \t %f \t ", sam->gyro_data[0][sam_n], sam->gyro_data[1][sam_n]);
		//���̌X���p�x(�W���C���Z���T)
		fprintf(fp_sensor, "%f \t %f \t ", sam->gyro_angle_deg[0][sam_n], sam->gyro_angle_deg[1][sam_n]);
		//���[�p�X�ʂ�����̓��̌X���p�x(�W���C���Z���T)
		fprintf(fp_sensor, "%f \t %f \t ", sam->gyro_angle_lowp_deg[0][sam_n], sam->gyro_angle_lowp_deg[1][sam_n]);
		//�����x�Z���T�o��
		fprintf(fp_sensor, "%f \t %f \t ", sam->acc_data[0][sam_n], sam->acc_data[1][sam_n]);
		//���̌X���p�x(�����x�Z���T)
		fprintf(fp_sensor, "%f \t %f \t ", sam->acc_angle_deg[0][sam_n], sam->acc_angle_deg[1][sam_n]);
		//���[�p�X�ʂ�����̓��̌X���p�x(�����x�Z���T)
		fprintf(fp_sensor, "%f \t %f \t ", sam->acc_angle_lowp_deg[0][sam_n], sam->acc_angle_lowp_deg[1][sam_n]);

		//�l�H�ؓ����C��
		fprintf(fp_sensor, "%f \t ", sam->atmosphere[sam_n]);

		fprintf(fp_sensor, "\n");	// ���s
	}

	//�������݃t�@�C���� ����
	fclose(fp_neuron);
	fclose(fp_sensor);
	return 0;
}

// ���Ԍo�߂ɂ��p�����[�^�ݒ�
void param(float d[2], LOADType *load, float t)
{
	int i;	// �J�E���^

	// ��ʓ��͂̕ύX
	if (t<Time_State1){	// �����p�����[�^
		d[0] = Sup_Driven1;
		d[1] = Ext_Driven1;
	}
	else if (t<Time_State2){	// �p�����[�^�ύX��
		d[0] = Sup_Driven1 + (Sup_Driven2 - Sup_Driven1)/(Time_State2 - Time_State1) * (t - Time_State1);
		d[1] = Ext_Driven1 + (Ext_Driven2 - Ext_Driven1)/(Time_State2 - Time_State1) * (t - Time_State1);
	}
	else{	// �p�����[�^�ύX��
		d[0] = Sup_Driven2;
		d[1] = Ext_Driven2;
	}

	// �l�H�ؓ��̒��͓����̕ύX
	for(i=0; i<UNITS; i++) {
		if(i == LF || i == RF) { // �O�r
			if (t<Time_State1)	// �������͓���
				load->load_char[i] = Load_char1_F;
			else if (t<Time_State2)	// ���͓����ύX��
				load->load_char[i] = Load_char1_F + (Load_char2_F - Load_char1_F)/(Time_State2 - Time_State1) * (t - Time_State1);
			else	// ���͓����ύX��
				load->load_char[i] = Load_char2_F;
		}
		else { // ��r
			if (t<Time_State1)	// �������͓���
				load->load_char[i] = Load_char1_H;
			else if (t<Time_State2)	// ���͓����ύX��
				load->load_char[i] = Load_char1_H + (Load_char2_H - Load_char1_H)/(Time_State2 - Time_State1) * (t - Time_State1);
			else	// ���͓����ύX��
				load->load_char[i] = Load_char2_H;
		}
	}
}


// �֐��̕��Ϗ������Ԃƍő又�����Ԃ�\��
void Calc_proc_time(int k, float t_p_sum, float t_p_max)
{
	float	t_p_ave;	// �֐��̕��Ϗ�������

	// ���Ϗ������Ԃ̌v�Z(�֐��̍��v�������� / �֐��̎��s��)
	t_p_ave = t_p_sum / (float)k;

	printf("Specified Function's ...\n");
	printf("\t Mean Processing Time : %f[ms]\n", t_p_ave * 1000.0);
	printf("\t Max Processing Time : %f[ms]\n\n", t_p_max * 1000.0);


	/* ���̊֐����g�p����ꍇ�̒��� --------------------------

	�������Ԃ��v���������֐���"function_a()"�Ƃ����ꍇ�C

	// -------------------------------------------------------------------------------------------- //
	//	t1 = (float)rdtsc();				// �֐��������O�̎���			//
	//	function_a();					// �֐������s				//
	//	t2 = (float)rdtsc();				// �֐���������̎���			//
	//	t_proc = (float)((t2-t1)/tmu_frq/1.0e+6);	// �֐��̏����ɂ����������Ԃ��v�Z	//
	//	if(t_proc > t_proc_max) t_proc_max = t_proc;	// �ő又�����Ԃ̍X�V			//
	//	k_func++;					// �֐����s�񐔍X�V			//
	// -------------------------------------------------------------------------------------------- //

	��L�̂悤�ȋL�q��"1_main.c"���ɒǉ�����K�v������

	------------------------------------------------------- */
}