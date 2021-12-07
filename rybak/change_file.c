
//      gcc change_file.c -o change_file
#include <stdio.h>

void fprintBi(int num,FILE *file);


int main(void){

    int i,j;

    //�ǂݍ��ރt�@�C���̃T�C�Y�w��
    int cell_num_n = 115; //log_muscle�̗�̐�
    int cell_num_s = 69; //lon_sensor�̗�̐�
    int cell_time = 6000;   //�s���̓ǂݍ��ݏ��

    //�ǂݍ��ރf�[�^�̎w��
    //neuron
    int cell_muscle_io = 115; //�ؓ�ONOFF���̃Z���ԍ�
    int cell_cpg = 3; //~6 //CPG���Η��̃Z���ԍ�
    int cell_Mn = 43;

    char name_data_n[cell_num_n][200]; //���ږ��̓ǂݍ��݃f�[�^
    char name_data_s[cell_num_s][200];
    float data_n[cell_num_n];   //1�s���̓ǂݍ��݃f�[�^_neuron
    float data_s[cell_num_s];   //1�s���̓ǂݍ��݃f�[�^_sensor
    int time_limit;
    int limit_flag = 0;

    //�f�[�^��ۑ�����ϐ�
    float time[cell_time];  //���Ԃ̕ۑ�
    unsigned int muscle_bit[cell_time]; //�ؓ�ONOFF���̕ۑ�
    float cpg[4][cell_time]; //CPG���Η��̕ۑ�
    float Mn[24][cell_time]; //Mn���Η��̕ۑ�
    int k;

    FILE	*fp_neuron = fopen("log_neuron.txt", "r");	//log_neuron�̃I�[�v��
    FILE	*fp_sensor = fopen("log_sensor.txt", "r");	//log_sensor�̃I�[�v��

    FILE	*fp_output = fopen("log_muscle.txt", "w");	//�ۑ��t�@�C���̃I�[�v��

    if (fp_neuron == NULL) {
        printf("�t�@�C�����g�p���ł�(neuron)�D\n");
        printf("\n*** press Enter to END ***\n");
        while(getchar() != '\n');
        return 1;
    }
    if (fp_sensor == NULL) {
        printf("�t�@�C�����g�p���ł�(sensor)�D\n");
        printf("\n*** press Enter to END ***\n");
        while(getchar() != '\n');
        return 1;
    }
    if (fp_output == NULL) {
        printf("�t�@�C�����g�p���ł�(output)�D\n");
        printf("\n*** press Enter to END ***\n");
        while(getchar() != '\n');
        return 1;
    }


//�f�[�^�̓ǂݍ���
    /**********  neuron  **********/
    //���ږ��̓ǂݍ���
    for(i=0; i<cell_num_n; i++){
		fscanf(fp_neuron, "%s", &name_data_n[i]);
	}
    //�f�[�^�̓ǂݍ���
    for(i=0; i<cell_time; i++){
        for(j=0; j<cell_num_n; j++){
		    if(fscanf(fp_neuron, "%f", &data_n[j]) == EOF) limit_flag = 1;  //EOF��ǂ񂾂�ǂݍ��ݏI��
        }
        for(j=0; j<cell_num_n; j++){
            if(cell_cpg-1 == j){
                for(k=0;k<4;k++){
                    cpg[k][i] = data_n[j+k];    //CPG���Η��̕ۑ�
                }
            }
            if(j == (cell_Mn-1)){
                for(k=0;k<24;k++){
                    Mn[k][i] = data_n[j+k];    //Mn���Η��̕ۑ�
                }
            }
    		if(j == (cell_muscle_io-1))
			    muscle_bit[i] = (int)data_n[j];    //�ؓ�ONOFF���̕ۑ�
	    }
        time[i] = data_n[0];  //���Ԃ̕ۑ�
        time_limit = i; //�ǂݍ��݂����s��
        if (limit_flag == 1) break;
    }
    fclose(fp_neuron);
    /******************************/
    /**********  sensor  **********/
    limit_flag = 0; //�I���t���O�̃��Z�b�g
    //���ږ��̓ǂݍ���
    for(i=0; i<cell_num_s; i++){
		fscanf(fp_sensor, "%s", &name_data_s[i]);
	}

    //�f�[�^�̓ǂݍ���
    for(i=0; i<cell_time; i++){
        for(j=0; j<cell_num_s; j++){
		    if(fscanf(fp_sensor, "%f", &data_s[j]) == EOF) limit_flag = 1;  //EOF��ǂ񂾂�ǂݍ��ݏI��


	    }
        if (limit_flag == 1) break;
    }
    fclose(fp_sensor);
    /******************************/


//�f�[�^�̏�������
    //���ږ�
    fprintf(fp_output,"time \t");   //����

    fprintf(fp_output,"CPG_LF \t CPG_LH \t CPG_RF \t CPG_RH \t");    //CPG

    fprintf(fp_output,"LF-1 \t LF-2 \t LF-3 \t LF-4 \t LF-5 \t LF-6 \t");    //�ؓ�ONOFF
    fprintf(fp_output,"LH-1 \t LH-2 \t LH-3 \t LH-4 \t LH-5 \t LH-6 \t");
    fprintf(fp_output,"RF-1 \t RF-2 \t RF-3 \t RF-4 \t RF-5 \t RF-6 \t");
    fprintf(fp_output,"RH-1 \t RH-2 \t RH-3 \t RH-4 \t RH-5 \t RH-6 \t");

    fprintf(fp_output,"LF-Mn1 \t LF-Mn2 \t LF-Mn3 \t LF-Mn4 \t LF-Mn5 \t LF-Mn6 \t");    //Mn
    fprintf(fp_output,"LH-Mn1 \t LH-Mn2 \t LH-Mn3 \t LH-Mn4 \t LH-Mn5 \t LH-Mn6 \t");
    fprintf(fp_output,"RF-Mn1 \t RF-Mn2 \t RF-Mn3 \t RF-Mn4 \t RF-Mn5 \t RF-Mn6 \t");
    fprintf(fp_output,"RH-Mn1 \t RH-Mn2 \t RH-Mn3 \t RH-Mn4 \t RH-Mn5 \t RH-Mn6 \t");

    fprintf(fp_output,"\n");


    //���f�[�^
    for(i=0; i<time_limit; i++){
        fprintf(fp_output,"%f\t",time[i]);  //����
        for(j=0;j<4;j++) fprintf(fp_output,"%f\t",cpg[j][i]); //CPG
        fprintBi(muscle_bit[i], fp_output); //�ؓ�ONOFF
        for(j=0;j<24;j++) fprintf(fp_output,"%f\t",Mn[j][i]); //Mn

        fprintf(fp_output,"\n");

    }


    fclose(fp_output);


}


//�ؓ�ONOFF�f�[�^���i���\�L�Ńt�@�C���o��
void fprintBi(int num, FILE *out_file)
{

    int unit = 4;
    int muscle = 6;

    int len = unit * muscle;
    int bit[len];
    int i;
    unsigned int x;

    for(int i = 0; i < len; i++) {
        x = 1 << i;
        x = num & x;
        bit[i] = x >> i;
    }

    for(int i = 0; i < len; i++) {
        fprintf(out_file,"%d\t", (i+1)*bit[i]);
    }
}
