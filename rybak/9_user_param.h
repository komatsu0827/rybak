/*�ύX�����ȃp�����[�^�������ɂ܂Ƃ߂�
�@�@���ӁF�����Œl��ݒ肵�Ă��Ă��{���̒��ŏ��������Ă���\�������邽�߂悭�m�F���邱�ƁD*/

#ifndef __USER_PARAM_HEADER_INCLUDED__
#define __USER_PARAM_HEADER_INCLUDED__

// �e����s���� --------------------------------------------------------------------------
//#define Time_Loop	(35.0)	// ���C�����[�v����[s]�i���s�I��������������v�Z���s���j
#define Time_Locomotion	(30.0)	// ���s����[s]
#define Time_Reset 	(5.0/*5.0*/)	// �S�ؓ��o�Ɏ���[s]
#define Time_Flexion	(5.0)	// �S�֐ߋ��Ȏ���[s]
#define Time_Extension	(5.0)	// �S�֐ߐL�W����[s]
#define Time_Offset	(1.0)	// �I�t�Z�b�g�v������[s]
#define TIME_STEP 	(1000)	// �T���v�����O�^�C���i1000[��sec] �� 1[msec]�j

// �e��o�ߎ��Ԑݒ� ----------------------------------------------------------------------
// CPG�g�`�����肳���邽�߁C���s�����i�o�Ɂ����ȁ��o�Ɂj����CPG�v�Z�͎��s���Ă���
#define Time_Step1	(Time_Offset)			// ���s�����i�I�t�Z�b�g�v���j
#define Time_Step2	(Time_Step1 + Time_Reset)	// ���s�����i��x�ڂ̑S�ؓ��o�Ɂj
#define Time_Step3	(Time_Step2 + Time_Flexion)	// ���s�����i�S�֐ߋ���&�G���R�[�_���Z�b�g�j
#define Time_Step4	(Time_Step3 + Time_Reset)	// ���s�����i��x�ڂ̑S�ؓ��o�Ɂj
#define Time_Step5	(Time_Step4 + Time_Locomotion)	// ���s�iTime_Step4:���s�J�n�����ɑ����j
#define Time_Step6	(Time_Step5 + Time_Reset)	// ���s��̑S�ؓ��o��

// �p�����[�^�ύX�̃^�C�~���O�ݒ�iTime_State1 < Time_State �ƂȂ�悤�Ɂj
#define Time_State1	(Time_Step4 + 10.0)	// �p�����[�^�ύX�J�n����
#define Time_State2	(Time_Step4 + 20.0)	// �p�����[�^�ύX�I������

// �r�Ԍ����׏d --------------------------------------------------------------------------
#define U2U_FH	(0.04/*/0.038*/)	// �}�����O���
#define U2U_LR	(0.04/*/0.032*/)	// �}�������E��

#define U1U_FH	(0.05/*/0.038*/)	// �������O���
#define U1U_LR	(0.05/*/0.032*/)	// ���������E��

// ��ʓ��͐ݒ� --------------------------------------------------------------------------
#define Sup_Driven1	(1.56)//����2.00 (1.70)//*/(1.96)	// ��ʓ���d[0]�i�p�����[�^�ύX�O�j2019/01/15 �t�B�[�h�o�b�N����:
#define Sup_Driven2	(1.56)//����2.00 (2.68)//*/(1.96)	// ��ʓ���d[0]�i�p�����[�^�ύX��j		����1.56���x�E�E�E����0.54
#define Ext_Driven1	(0.0)	// �O������d[1]�i�p�����[�^�ύX�O�j
#define Ext_Driven2	(0.0)	// �O������d[1]�i�p�����[�^�ύX��j

// �ڒn����ɗp����FF�d���l��臒l
#define FF_touch_th	(3.0)

// �ڒn����ɗp���鑫��v���[�g���Ίp�x��臒l
#define POT_touch_th	(5.0)

// �ؒ��͓����ݒ�i�ؒ��̕ψʊ����ɑ΂��钣�́j [N]��[kN]�ɕϊ����� ------------------------------------------
/***�p�����[�^�ڈ�****************************************
        (1.28)  0.3[MPaG]
        (1.56)  0.35[MPaG]
        (1.85)  0.4[MPaG]
        (2.17)  0.45[MPaG]
        (2.49)  0.5[MPaG]
        (2.66)  0.55[MPaG]
        (2.84)  0.6[MPaG]
********************************************************/
//�O�r
#define Load_char1_F	(2.49) //(2.49) 		//0.53MPaG	//0.3MPaG:1.28, 0.4MPaG:1.85
#define Load_char2_F	(2.49) //(2.84)		//0.65MPaG	//0.5MPaG:2.49, 0.6MPaG:2.84
//��r
#define Load_char1_H	(Load_char1_F) //(2.49)		//0.53MPaG	//0.3MPaG:1.28, 0.4MPaG:1.85
#define Load_char2_H	(Load_char2_F) //(2.84)		//0.65MPaG	//0.5MPaG:2.49, 0.6MPaG:2.84//#define Load_char2	(2.8402)	// �p�����[�^�ύX��(0.6[MPaG])

// �r���׃Q�C���i�O�r�ƌ�r�ŋr���ׂ̑傫�����قȂ�̂ŗv�����j --------------------------
#define load_gain_H	(20.0)	//(20.0)	// ��r
#define load_gain_F	(load_gain_H) 	// �O�r

#define sigmoid_alpha	(3.00) //0.75

// Mn�̐U���ɂ��d���ِ��� 臒l --------------------------
// �d���ق��쓮������Mn���Η���臒l
#define Mn_fVth		(0.03)
//�d���ى���i�K��臒l
#define Mn_phase0 (0.03)    //[0 0 0]��[1 0 0]�̋��E
#define Mn_phase1 (0.07)    //[1 0 0]��[1 1 0]�̋��E
#define Mn_phase2 (0.17)    //[1 1 0]��[1 1 1]�̋��E

// �v�Z�������̂��߂Ɏg�p����p�����[�^
#define	Muscle_len_table_num	(101)	// �ؒ��v�Z�e�[�u���̕�����
#define V_num	(1001)	// �j���[�������p�����[�^�v�Z�e�[�u���̕�����
#define V_MAX	(30.0)	// �j���[�������d�ʂ̍ő�l
#define V_MIN	(-80.0)	// �j���[�������d�ʂ̍ŏ��l


// ������ �n�[�h�ɕύX���������ꍇ ������
// �e�����N��[mm] ------------------------------------------------------------------------
#define Link1_F_Length	(64.0)	// �O�r�����N1
#define Link2_F_Length	(130.0)	// �O�r�����N2
#define Link3_F_Length	(161.0)	// �O�r�����N3

#define Link1_H_Length	(150.0)	// ��r�����N1
#define Link2_H_Length	(150.0)	// ��r�����N2
#define Link3_H_Length	(91.5)	// ��r�����N3

// �ؓ��̎��t���ʒu�i�֐ߎ��Ƃ̋���[mm]�j ----------------------------------------------
// �O�r1�ԋ�
#define Muscle1_F_Connect_ax	(20.0)
#define Muscle1_F_Connect_ay	(95.0)
#define Muscle1_F_Connect_bx	(-20.0)
#define Muscle1_F_Connect_by	(10.0)
// �O�r2�ԋ�
#define Muscle2_F_Connect_ax	(-130.0)
#define Muscle2_F_Connect_ay	(55.0)
#define Muscle2_F_Connect_bx	(-30.0)
#define Muscle2_F_Connect_by	(-10.0)
// �O�r3�ԋ�
#define Muscle3_F_Connect_ax	(-45.0)
#define Muscle3_F_Connect_ay	(60.0)
#define Muscle3_F_Connect_bx	(18.0)
#define Muscle3_F_Connect_by	(0.0)
// �O�r4�ԋ�
#define Muscle4_F_Connect_ax	(-60.0)
#define Muscle4_F_Connect_ay	(20.0)
#define Muscle4_F_Connect_bx	(-30.0)
#define Muscle4_F_Connect_by	(10.0)
// �O�r5�ԋ�
#define Muscle5_F_Connect_ax	(110.0)
#define Muscle5_F_Connect_ay	(3.0)
#define Muscle5_F_Connect_bx	(130.0)
#define Muscle5_F_Connect_by	(10.0)
// �O�r6�ԋ�
#define Muscle6_F_Connect_ax	(-15.0)
#define Muscle6_F_Connect_ay	(-8.0)
#define Muscle6_F_Connect_bx	(-20.0)
#define Muscle6_F_Connect_by	(-11.0)
// ��r1�ԋ�
#define Muscle1_H_Connect_ax	(20.0)
#define Muscle1_H_Connect_ay	(0.0)
#define Muscle1_H_Connect_bx	(-17.0)
#define Muscle1_H_Connect_by	(9.5)
// ��r2�ԋ�
#define Muscle2_H_Connect_ax	(-50.0)
#define Muscle2_H_Connect_ay	(4.0)
#define Muscle2_H_Connect_bx	(-27.0)
#define Muscle2_H_Connect_by	(-10.0)
// ��r3�ԋ�
#define Muscle3_H_Connect_ax	(-10.0)
#define Muscle3_H_Connect_ay	(10.0)
#define Muscle3_H_Connect_bx	(21.0)
#define Muscle3_H_Connect_by	(0.0)
// ��r4�ԋ�
#define Muscle4_H_Connect_ax	(-15.0)
#define Muscle4_H_Connect_ay	(12.0)
#define Muscle4_H_Connect_bx	(-27.0)
#define Muscle4_H_Connect_by	(0.0)
// ��r5�ԋ�
#define Muscle5_H_Connect_ax	(21.0)
#define Muscle5_H_Connect_ay	(0.0)
#define Muscle5_H_Connect_bx	(20.0)
#define Muscle5_H_Connect_by	(2.0)
// ��r6�ԋ�
#define Muscle6_H_Connect_ax	(-13.0)
#define Muscle6_H_Connect_ay	(-5.0)
#define Muscle6_H_Connect_bx	(-27.0)
#define Muscle6_H_Connect_by	(-10.0)


// ��3�֐ߐL�؂̍ő�ؒ� -----------------------------------------------------------------
#define Elbow_Extensor_Length_MAX	(150)	// �I�L��
#define Ankle_Extensor_Length_MAX	(180)	// ����L��

//�e�֐߂̍ŏ��p�x�i�ő���Ȏ��j[deg] ----------------------------------------------------
// �O�r
#define	ANGLE_SCAPULA_MIN	(40.0)	// ���b��
#define	ANGLE_SHOULDER_MIN	(80.0)	// ��
#define	ANGLE_ELBOW_MIN		(65.0)	// �I
// ��r
#define	ANGLE_HIP_MIN		(40.0)	// ��
#define	ANGLE_KNEE_MIN		(75.0)	// �G
#define	ANGLE_ANKLE_MIN		(40.0)	// ����

//�e�֐߂̍ő�p�x�i�ő�L�W���j[deg] ----------------------------------------------------
// �O�r
#define ANGLE_SCAPULA_MAX	(85.0)	// ���b��
#define	ANGLE_SHOULDER_MAX	(120.0)	// ��
#define	ANGLE_ELBOW_MAX		(125.0)	// �I
// ��r
#define	ANGLE_HIP_MAX		(105.0)	// ��
#define	ANGLE_KNEE_MAX		(120.0)	// �G
#define	ANGLE_ANKLE_MAX		(120.0)	// ����



#endif