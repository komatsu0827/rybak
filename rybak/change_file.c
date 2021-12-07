
//      gcc change_file.c -o change_file
#include <stdio.h>

void fprintBi(int num,FILE *file);


int main(void){

    int i,j;

    //読み込むファイルのサイズ指定
    int cell_num_n = 115; //log_muscleの列の数
    int cell_num_s = 69; //lon_sensorの列の数
    int cell_time = 6000;   //行数の読み込み上限

    //読み込むデータの指定
    //neuron
    int cell_muscle_io = 115; //筋肉ONOFF情報のセル番号
    int cell_cpg = 3; //~6 //CPG発火率のセル番号
    int cell_Mn = 43;

    char name_data_n[cell_num_n][200]; //項目名の読み込みデータ
    char name_data_s[cell_num_s][200];
    float data_n[cell_num_n];   //1行分の読み込みデータ_neuron
    float data_s[cell_num_s];   //1行分の読み込みデータ_sensor
    int time_limit;
    int limit_flag = 0;

    //データを保存する変数
    float time[cell_time];  //時間の保存
    unsigned int muscle_bit[cell_time]; //筋肉ONOFF情報の保存
    float cpg[4][cell_time]; //CPG発火率の保存
    float Mn[24][cell_time]; //Mn発火率の保存
    int k;

    FILE	*fp_neuron = fopen("log_neuron.txt", "r");	//log_neuronのオープン
    FILE	*fp_sensor = fopen("log_sensor.txt", "r");	//log_sensorのオープン

    FILE	*fp_output = fopen("log_muscle.txt", "w");	//保存ファイルのオープン

    if (fp_neuron == NULL) {
        printf("ファイルが使用中です(neuron)．\n");
        printf("\n*** press Enter to END ***\n");
        while(getchar() != '\n');
        return 1;
    }
    if (fp_sensor == NULL) {
        printf("ファイルが使用中です(sensor)．\n");
        printf("\n*** press Enter to END ***\n");
        while(getchar() != '\n');
        return 1;
    }
    if (fp_output == NULL) {
        printf("ファイルが使用中です(output)．\n");
        printf("\n*** press Enter to END ***\n");
        while(getchar() != '\n');
        return 1;
    }


//データの読み込み
    /**********  neuron  **********/
    //項目名の読み込み
    for(i=0; i<cell_num_n; i++){
		fscanf(fp_neuron, "%s", &name_data_n[i]);
	}
    //データの読み込み
    for(i=0; i<cell_time; i++){
        for(j=0; j<cell_num_n; j++){
		    if(fscanf(fp_neuron, "%f", &data_n[j]) == EOF) limit_flag = 1;  //EOFを読んだら読み込み終了
        }
        for(j=0; j<cell_num_n; j++){
            if(cell_cpg-1 == j){
                for(k=0;k<4;k++){
                    cpg[k][i] = data_n[j+k];    //CPG発火率の保存
                }
            }
            if(j == (cell_Mn-1)){
                for(k=0;k<24;k++){
                    Mn[k][i] = data_n[j+k];    //Mn発火率の保存
                }
            }
    		if(j == (cell_muscle_io-1))
			    muscle_bit[i] = (int)data_n[j];    //筋肉ONOFF情報の保存
	    }
        time[i] = data_n[0];  //時間の保存
        time_limit = i; //読み込みした行数
        if (limit_flag == 1) break;
    }
    fclose(fp_neuron);
    /******************************/
    /**********  sensor  **********/
    limit_flag = 0; //終了フラグのリセット
    //項目名の読み込み
    for(i=0; i<cell_num_s; i++){
		fscanf(fp_sensor, "%s", &name_data_s[i]);
	}

    //データの読み込み
    for(i=0; i<cell_time; i++){
        for(j=0; j<cell_num_s; j++){
		    if(fscanf(fp_sensor, "%f", &data_s[j]) == EOF) limit_flag = 1;  //EOFを読んだら読み込み終了


	    }
        if (limit_flag == 1) break;
    }
    fclose(fp_sensor);
    /******************************/


//データの書き込み
    //項目名
    fprintf(fp_output,"time \t");   //時間

    fprintf(fp_output,"CPG_LF \t CPG_LH \t CPG_RF \t CPG_RH \t");    //CPG

    fprintf(fp_output,"LF-1 \t LF-2 \t LF-3 \t LF-4 \t LF-5 \t LF-6 \t");    //筋肉ONOFF
    fprintf(fp_output,"LH-1 \t LH-2 \t LH-3 \t LH-4 \t LH-5 \t LH-6 \t");
    fprintf(fp_output,"RF-1 \t RF-2 \t RF-3 \t RF-4 \t RF-5 \t RF-6 \t");
    fprintf(fp_output,"RH-1 \t RH-2 \t RH-3 \t RH-4 \t RH-5 \t RH-6 \t");

    fprintf(fp_output,"LF-Mn1 \t LF-Mn2 \t LF-Mn3 \t LF-Mn4 \t LF-Mn5 \t LF-Mn6 \t");    //Mn
    fprintf(fp_output,"LH-Mn1 \t LH-Mn2 \t LH-Mn3 \t LH-Mn4 \t LH-Mn5 \t LH-Mn6 \t");
    fprintf(fp_output,"RF-Mn1 \t RF-Mn2 \t RF-Mn3 \t RF-Mn4 \t RF-Mn5 \t RF-Mn6 \t");
    fprintf(fp_output,"RH-Mn1 \t RH-Mn2 \t RH-Mn3 \t RH-Mn4 \t RH-Mn5 \t RH-Mn6 \t");

    fprintf(fp_output,"\n");


    //実データ
    for(i=0; i<time_limit; i++){
        fprintf(fp_output,"%f\t",time[i]);  //時間
        for(j=0;j<4;j++) fprintf(fp_output,"%f\t",cpg[j][i]); //CPG
        fprintBi(muscle_bit[i], fp_output); //筋肉ONOFF
        for(j=0;j<24;j++) fprintf(fp_output,"%f\t",Mn[j][i]); //Mn

        fprintf(fp_output,"\n");

    }


    fclose(fp_output);


}


//筋肉ONOFFデータを二進数表記でファイル出力
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
