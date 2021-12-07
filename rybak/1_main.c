// ---------------------------------------------------- //
//	�v���O����"Rybak_robot"�̃��C���v���O����	//
//			main.c				//
// ---------------------------------------------------- //

#include "8_define.h"
#include "9_user_param.h"

int main(void)
{
	//------�ϐ��錾�Ə�����------//
	int		DIO_MODE;				// DIO���[�h�ݒ�p
	double		AD_data[num_AD];			// AD�|�[�g�o�͂̊i�[��(HRP��API�d�l�Ɋ�Â�double�^)
	int		i;					// �ėp�J�E���^
	int		k_offset = 0;				// �I�t�Z�b�g�v���p�J�E���^
	int		k_sam = 0;				// �T���v���f�[�^�p�J�E���^
	int		k_func = 0;				// �֐����s�񐔗p�J�E���^
	float		d[2] = {Sup_Driven1, Ext_Driven1};	// ��ʓ���
	float		t = 0.0;				// ���s�J�n��̌o�ߎ���[sec]
	float		t_last = 0.0;				// ���C�����[�v�̍ŏI���s����[sec]
	float		t_last_1 = 0.0;				// ����1�̍ŏI���s����[sec]
    float		t_last_2 = 0.0;				// ����2�̍ŏI���s����[sec]
	float 		ts = 0.0;				// ���s�J�n����
	float 		tn = 0.0;				// ���ݎ���
	float		t1;					// �֐��������O�̎��� (�֐��������Ԍv���p)
	float		t2;					// �֐���������̎���	      �V
	float		t_proc;					// �֐��̏�������[s]
	float		t_proc_sum = 0.0;			// �֐��̍��v��������[s]
	float		t_proc_max = 0.0;			// �֐��̍ő又������[s]
	double		tmu_frq = tsc_clock();			// �^�C�}���j�b�g�̎��g��
	int		flag_step = 0;				// ���[�v���̃X�e�b�v�؂�ւ��t���O

	// �e��\���̐錾
	RybakUnit	unit[UNITS];
	ENCType		enc;
	ADType		ad;
	SAMType		sam;
	LOADType	load;

	// ���j�b�g������
	InitUnit(unit);

	//------���b�Z�[�W�\��------//
	printf("\t \t Program Start!\n");	// �v���O�����J�n���b�Z�[�W�\��

	// �v�Z�������̂��߂̃e�[�u���쐬
	printf("\t \t Make Look-Up Tables\n");
	Muscle_len_table_init();
	Aux_table_init();
	fV_table_init();

	//------�J�n����------//
	openHRP3sh4();	// HRP�f�o�C�X�g�p�J�n
	tsc_open();	// �^�C�}���j�b�g�J�E���^�����[�U��

	//------���A���^�C������J�n------//
	// ART_PRIO_MAX:�ŗD��, ART_TASK_PERIODIC:�������s, TIME_STEP:�T���v�����O�^�C��
	if(art_enter(ART_PRIO_MAX, ART_TASK_PERIODIC, TIME_STEP) != 0){
		perror("art_enter");
		return -1;
	}

	//------A/D�ϊ��J�n------//
	startAD();

	//------������------//
	// PWM�̏�����
	for (i=0; i<num_PWM; i++) {
		setPWMmode(i, ON);	// PWM��S��ON
		setPWMduty(i, 0.0);
	}
	// DIO�̏�����
	DIO_MODE = 0xffffffff;
	setDIOmodeAll32(DIO_MODE);	// DIO���[�h�̐ݒ�
	for (i=0; i<num_DIO; i++)
		setDIO(i, OFF);		// DIO��S��OFF

	// ���ٍ쓮�i��肭�ʐM�ł��Ȃ���������̂ŁC��������s���Ă����j
	udp_transmission(t);
	udp_transmission(t);
	udp_transmission(t);

	ts = tn = (float)rdtsc(); // ���[�v�J�n�����ݒ�

	// ���C�����[�v ////////////////////////////////////////////////////////////////////////////////////////////////
	while (1) {
		t1 = (float)rdtsc();				// �֐��������O�̎���

		//----�������s������̎擾----//
		if(art_wait() != 0)
			perror("art_wait");

		// �����X�V
		tn = (float)rdtsc();	// ���ݎ����擾
		t = (float)((tn-ts)/tmu_frq/1.0e+6) + 2.0e-3;	// �o�ߎ��Ԃ̍X�V


		// �X�e�b�v�p�t���O�̍X�V���X�e�b�v�ڍs���b�Z�[�W�̕\�� --------------- //
		// �X�e�b�v1�i�I�t�Z�b�g�v�����ԁj�ֈڍs
		if (flag_step == 0) {
			flag_step++;
			printf("t = %f\n", t);
			printf("\t \t Measuring Offsets...\n\n");
		}
		// �I�t�Z�b�g�����肵�X�e�b�v2�i�S�ؓ��o�Ɂj�ֈڍs
		if(flag_step == 1 && t > Time_Step1) {
			measure_offset_con(k_offset, &ad);	// �I�t�Z�b�g�v��(�I�t�Z�b�g����p)
			flag_step++;
			printf("t = %f\n", t);
			printf("\t \t Resetting Muscles...\n\n");
		}
		// �G���R�[�_�������̂��߂ɃX�e�b�v3�i�S�֐ߋ��ȁj�ֈڍs
		if (flag_step == 2 && t > Time_Step2) {
			flag_step++;
			printf("t = %f\n", t);
			printf("\t \t Setting Encoder...\n\n");
		}
		// �G���R�[�_���������s���X�e�b�v4�i�S�ؓ��o�Ɂj�ֈڍs
		if (flag_step == 3 && t > Time_Step3) {
			for(i=0; i<num_ENC; i++)
				setEnc(i,0);	// �G���R�[�_������
			flag_step++;
			printf("t = %f\n", t);
			printf("\t \t Resetting Muscles...\n\n");
		}
		// �X�e�b�v5�i���s�^���j�ֈڍs
		if (flag_step == 4 && t > Time_Step4) {
			flag_step++;
			printf("t = %f\n", t);
			printf("\t \t Locomotion Start!\n\n");
			t_last_1 = t;
            t_last_2 = t;
		}
		// ���s�^�����I�����C�X�e�b�v6�i�S�ؓ��o�Ɂj�ֈڍs
		if (flag_step == 5 && t > Time_Step5) {
			udp_transmission(t);	// ���ْ�~�i�O�̂��ߕ�������s�j
			udp_transmission(t);
			udp_transmission(t);
			printf("t = %f\n", t);
			printf("\t \t Locomotion End.\n\n");
			flag_step++;
			printf("\t \t Resetting Muscles...\n\n");
		}
		// ���C�����[�v���甲���o��
		if (flag_step == 6 && t > Time_Step6)
			break;
		// -------------------------------------------------------------------- //


		// CPG���̊e�j���[������Ԃ��X�V�i�X�e�b�v�Ɋւ�炸�����[�v���s�j
		//����2����4���ǂ��炩����̓R�����g�A�E�g���邱��
		CalcUnit_RK2(unit, d, t-t_last, &enc, &load);	// 2�������Q�N�b�^
//		CalcUnit_RK4(unit, d, t-t_last, &enc, &load);	// 4�������Q�N�b�^
		CalcCycle(unit, t);	// CPG�����̌v�Z


		// �X�e�b�v���Ƀ��[�v ------------------------------------------------------------------------
		switch (flag_step) {
			// ------ �X�e�b�v1�i�I�t�Z�b�g�v�����ԁj ------ //
			case 1:
				measure_offset_sum(&ad);	// �I�t�Z�b�g�v��(���Z�p)
				k_offset++;				// ���Z�񐔍X�V
				break;
			// ------ �X�e�b�v2�i�S�ؓ��o��1�j ------ //
			case 2:
				muscle_reset();		// �S�ؓ��o��
				break;
			// ------ �X�e�b�v3�i�S�֐ߋ��ȁj ------ //
			case 3:
				muscle_flexion();	// �S�֐ߋ���
				break;
			// ------ �X�e�b�v4�i�S�ؓ��o��2�j ------ //
			case 4:
				muscle_reset();		// �S�ؓ��o��
				break;
			// ------ �X�e�b�v5�i���s�^���j ------ //
			case 5:
				// �ؓ�����
				//control_Muscles(unit);
				// ���[�v�̃^�C�~���O�Ŏ��s���e�𕪊�---------------------------------- //
				//	�� �v�Z������2[ms]�Ɏ��߂邽�ߓ����ɂ͎��s���Ȃ�
				// ����1�F�@10[ms]���Ɏ��s
				if(1.0e-2 < t - t_last_1){
//					// �T���v���f�[�^�̕ۑ�
					data_get(k_sam, t, &sam, unit, &enc, &ad, &load); // �f�[�^�̊i�[
					k_sam++;	// �T���v���f�[�^�̗v�f���X�V
					// ���Ԍo�߂ɂ��p�����[�^�ݒ�
					param(d, &load, t);
					// ���Ԍo�߂ɂ����ق�duty��ݒ�
					udp_transmission(t);
					// ����1�̍ŏI���s�����̍X�V
					t_last_1 = t;
				}

				// ����2�F ��L�ȊO�̃^�C�~���O�ŏ�Ɏ��s
				else {
					//�G���R�[�_�ɂ��e�r�̊֐ߊp�x���
					all_angle_get(&enc);
					// �SAD�|�[�g�o�͒l�̎擾
					getADall(AD_data);	// getAD()�ɂ�钀���擾����getADall()�ɂ��ꊇ�擾�̕�������
					// FlexiForce�ɂ��ڒn���擾
					all_FF_get(&ad, AD_data);
					// �|�e���V�����[�^�ɂ��ڒn���擾
					all_POT_get(&ad, AD_data, t-t_last_2);
					// �ڒn����
					touch_get(&ad, &load);
					// �W���C���Z���T�ɂ�铷�̌X���p�x�擾
					gyro_get(&ad, AD_data, t-t_last_2);
					// �����x�Z���T�ɂ�铷�̌X���p�x�擾
					acc_get(&ad, AD_data, t-t_last_2);
					// ���̌X���p�x����
					tilt_con(&ad);
					// ��C���Z���T�ɂ��l�H�ؓ����C���̎擾
					air_press_get(&ad, AD_data);
					//�d���ِ���
					control_Muscles(t, unit);
					// �T���v���f�[�^�̕ۑ�
//					data_get(k_sam, t, &sam, unit, &enc, &ad, &load); // �f�[�^�̊i�[
//					k_sam++;	// �T���v���f�[�^�̗v�f���X�V
					// ����3�̍ŏI���s�����̍X�V
					t_last_2 = t;
				}
				// -------------------------------------------------------------------- //
				break;
			// ------ �X�e�b�v6�i�S�ؓ��o��3�j ------ //
			case 6:
				muscle_reset();		// �S�ؓ��o��
				break;
		}
		// -------------------------------------------------------------------------------------------


		speed_treadmill_get(&enc, t-t_last);	// �g���b�h�~�����x�擾

//		if(t-t_last > 2.5e-3)
//			printf("%f\n", t-t_last);	// �v�Z�����\��
		t_last = t;

		t2 = (float)rdtsc();				// �֐���������̎���
		t_proc = (float)((t2-t1)/tmu_frq/1.0e+6);	// �֐��̏����ɂ����������Ԃ��v�Z
		t_proc_sum += t_proc;				// �֐��̍��v�������Ԃ��X�V
		if(t_proc > t_proc_max) t_proc_max = t_proc;	// �ő又�����Ԃ̍X�V
		k_func++;					// �֐����s�񐔍X�V
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//------�I������------//
	// PWM��OFF
	for(i=0; i<num_PWM; i++){
		setPWMduty(i, 0.0);	// duty���S��0.0
		setPWMmode(i, OFF);	// PWM��S��OFF
	}
	// DIO��OFF
	for(i=0; i<num_DIO; i++)
		setDIO(i, OFF);		// DIO��S��OFF

	art_exit();		// ���A���^�C������I��
	closeHRP3sh4();		// �f�o�C�X�g�p�I��

	//------�t�@�C����������------//
	printf("\t \t Writing......\n\n");		// �t�@�C���������ݒ����b�Z�[�W�\��
	file_output(k_sam, &sam);			// �t�@�C���փf�[�^����������

	// �w�肵���֐��̕��ρE�ő又�����Ԃ̕\��
	Calc_proc_time(k_func, t_proc_sum, t_proc_max);

	printf("\t \t All Program End.\n\n");	// �v���O�����I�����b�Z�[�W�\��

	return 0;
}
