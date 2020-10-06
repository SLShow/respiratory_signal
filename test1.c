
#include<stdio.h>
#include<stdbool.h>
#include<stdlib.h>
#include<unistd.h>
//#include<Windows.h>
#include<math.h>
#define _CRT_SECURE_NO_WARNINGS；
#define INPUT_DATA_LEN 96
#define DATA_GROUP_COUNTS  10
#define VAR_THRESHOLD 5000    //方差门限

double varriance(double temp[]);
double getRespiratoryRate(double tem[], int dataLength);

int fs = 300;   //采样频率
#define  THRESHOLD   25     //门限     这个定值会存在问题
int win = THRESHOLD;       // 比例值
double max = -80000;
double min = 80000;
bool bDown = true;
bool bUp = false;
int peakIndex = 0;
int lastPeakIndex = 0;
int RespTimeLen = 0;
int lastRespTimeLen = 0;
float MinDiff = THRESHOLD;
int step_num = 0;
int MinIndex = 0;

static int peakAck_point[5];  //峰值点
static int respWaveLens[5];//呼吸波宽度
int i = 0;
int j = 0;   //分段点下标
int k = 0; //buffer下标
int residual_index = 0; //剩余数据的下标起始点
int count_b = 0; //buffer计数
int count_b_1 = 0;
int count_r = 0; //resp计数

double temp[INPUT_DATA_LEN * DATA_GROUP_COUNTS];  //暂存数组, 上一次传输剩余的数据
double var = 0;  //方差

double buffer[INPUT_DATA_LEN * DATA_GROUP_COUNTS];  //缓冲数组
double buffer_var[INPUT_DATA_LEN];    //缓存用于求方差的数据

bool buffer_full = false;  //buffer数组满标志
int residual_datalength = 0;  //剩余数组长度
bool residual = false;  //有剩余标志

/**********************计算呼吸频率**********************/
double getRespiratoryRate(double tem[], int dataLength) {     //数据与数据长度

    if (residual) {
        for (int i = 0; i < residual_datalength; i++) {
            buffer[i] = temp[i];
        }
        residual = false;
    }

    count_b++;
    count_b_1 = count_b - 1;

    for (k = 0; k < dataLength; k++) {  //每96*10个数据为一组进行处理
        buffer[residual_datalength + count_b_1 * dataLength + k] = tem[k];
        if (residual_datalength + count_b_1 * dataLength + k == INPUT_DATA_LEN * DATA_GROUP_COUNTS - 1) {  //buffer数组已满，跳出循环，将剩下未读完数据存储
            residual_index = k;
            for (i = 0; i < DATA_GROUP_COUNTS; i++) {
                for (int m = 0; m < INPUT_DATA_LEN; m++) {
                    buffer_var[m] = buffer[i * INPUT_DATA_LEN + m];
                }

                var = varriance(buffer_var);
                if (var == 0)
                    return -1;

                if (var >= VAR_THRESHOLD) {  //方差大于一定值则丢弃本段数据  重置各参数
                    max = -80000;
                    min = 80000;
                    bDown = true;
                    bUp = false;
                    peakIndex = 0;
                    lastPeakIndex = 0;
                    /*RespTimeLen = 0;
                    lastPeakIndex = 0;*/

                    //k = 0;
                    //var = 0;

                    count_b = 0;
                    count_r = 0;

                    break;
                }
            }
            if (var < VAR_THRESHOLD) {
                buffer_full = true;

                residual = true;
                residual_datalength = dataLength - residual_index - 1;
                residual_index = residual_index + 1;
                for (int p = 0; residual_index < dataLength; p++, residual_index++) {
                    temp[p] = tem[residual_index];
                }
            }
            break;
        }
    }

    if (buffer_full) {
        count_b = 0;
        count_r++;
        buffer_full = false;

        for (int i = 0; i < INPUT_DATA_LEN * DATA_GROUP_COUNTS; i++)
        {
            if (bDown) {
                if (buffer[i] > max)
                    max = buffer[i];
                if (max - buffer[i] > MinDiff) {      //下降达到一定值
                    step_num += 1;
                    if (i + (count_r - 1) * (INPUT_DATA_LEN * DATA_GROUP_COUNTS) - peakIndex > 450) {   //分段点距上一个分段点有一定距离  300为最小峰值间隔（1s）
                        bUp = true;
                        bDown = false;
                        min = buffer[i];
                        peakIndex = i + (count_r - 1) * (INPUT_DATA_LEN * DATA_GROUP_COUNTS);
                        RespTimeLen = peakIndex - lastPeakIndex;
                        if (RespTimeLen < 450 || RespTimeLen >3000)  //300HZ采样
                            RespTimeLen = lastRespTimeLen;
                        //printf("%d      ", lastPeakIndex);

                        if (j < 5) {
                            peakAck_point[j] = peakIndex;
                            respWaveLens[j] = RespTimeLen;
                        }
                        else {
                            for (int l = 0; l < 4; l++) {
                                peakAck_point[l] = peakAck_point[l + 1];     //记录分段点和段长
                                respWaveLens[l] = respWaveLens[l + 1];
                            }
                            peakAck_point[4] = peakIndex;
                            respWaveLens[4] = RespTimeLen;

                            //消除异常点
                            for (int l = 0;l < 3;l++) {
                                if (fabs(respWaveLens[l] - respWaveLens[l + 1]) >= 900 && fabs(respWaveLens[l] - respWaveLens[l + 2]) < 600)
                                    respWaveLens[l + 1] = (respWaveLens[l] + respWaveLens[l + 2]) / 2;
                            }
                        }
                        lastPeakIndex = peakIndex;
                        lastRespTimeLen = RespTimeLen;
                        j++;
                    }
                }
            }

            if (bUp) {
                if (buffer[i] < min) {
                    min = buffer[i];
                    MinIndex = i + (count_r - 1) * (INPUT_DATA_LEN * DATA_GROUP_COUNTS);
                }
                if (buffer[i] - min > MinDiff) {
                    if (MinIndex - peakIndex > 100) {
                        bUp = false;
                        bDown = true;
                        if ((max - min) / 10 < MinDiff  * 3  && (max - min) / 10 > MinDiff / 2) {
                            MinDiff = MinDiff * (win - 1) / win + (max - min) / 10 / win;
                        }
                        max = buffer[i];
                    }
                    else {
                        min = buffer[i];
                    }
                }
            }

            if (peakIndex - lastPeakIndex > 300 * 10 || peakIndex - lastPeakIndex < 450)
                MinDiff = THRESHOLD;
        }
    }


    i = 0;
    float aver_seg_length = 0;
    float rr = 0;
    while (i < 4 && respWaveLens[i] != 0) {
        //printf("%d  ", seg_length[i]);
        aver_seg_length += respWaveLens[i];
        i++;
    }
    aver_seg_length /= i;
    rr = 300 * 60 / aver_seg_length;
    if (isnan(rr))
        rr = -1;
    //printf("%f\n", rr);
    return rr;
}


/********计算方差**********/
double varriance(double tem[]) {
    double sum = 0;
    int length = 0;
    double average = 0;
    double var = 0;

    //length = sizeof(tem) / sizeof(tem[0]);
    length = 96;
    for (int n = 0;n < length;n++) {
        sum += tem[n];
    }
    average = sum / length;
    for (int p = 0;p < length;p++) {
        var += pow(tem[p] - average, 2) / length;
    }
    return var;
}


#define inputlength 96
int main() {
    static double resp_data[1000000];
    int i = 0;
    int count_print = 0;
    int test_j = 0;
    double rr = 0;
    double test[inputlength];
    FILE* fp;
    fp = fopen("/Users/slshaw/Desktop/test/test/1.txt", "r");
    if (fp == NULL)
        return -1;

    while (i <= 1000000)
    {
        fscanf(fp, "%lf,", &resp_data[i]);
        i++;
    }
    fclose(fp);
    while (test_j<1870) {
        i = 0;
        while (i < inputlength) {
            test[i] = resp_data[test_j * inputlength + i];
            i++;
        }
        test_j++;
        rr = getRespiratoryRate(test, 96);
        count_print++;
        //sleep(10);
        if (count_print == 10) {
            //printf("%lf\n", rr);
            printf("\n%d,  %d,  %d,  %d,  %lf\n", count_r, RespTimeLen, lastRespTimeLen, peakIndex, rr);
            count_print = 0;
        }
    }
    return 0;
}

