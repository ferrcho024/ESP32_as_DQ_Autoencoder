//
// Created by Usuario on 12/4/2023.
//
#include <math.h>
#include <stdio.h>

extern "C" {
#include "utils.h"
};

// float calculateMean(const float *data, size_t size) {
//     float sum = 0.0, d1;

//     for (int i = 0; i < size; i++) {
//     // Data 1
//     if(isnan(*(data + i))) {
//         d1 = 0;
//     }
//     else {
//         d1 = *(data + i);
//     }
//     sum += d1;
//     }

//     return sum / size;
// }


float completeness(float *data, int size) {
    int expectedValues = size;
    int validValues = 0;
    for (int i = 0; i < size; i++) {
        if (!isnan(*(data + i))) {
            validValues++;
            // Dato presente
#ifdef DEBUG
            printf("Data[%d] = %.4f\n", i,*(data + i));
#endif
        }
#ifdef DEBUG
        else {
            // Dato no disponible
            printf("Data[%d] = nan\n", i);
        }
#endif
    }
    return (float) validValues / expectedValues;
}

float uncertainty(float *data1,float *data2, int size) {
    float error = 0, d1, d2;
    float avg = 0;
    float v;
    float max;
    for (int i = 0; i < size; i++) {
        // Data 1
        if(isnan(*(data1 + i))) {
            d1 = 0;
        }
        else {
            d1 = *(data1 + i);
        }
        // Data 2
        if(isnan(*(data2 + i))) {
            d2 = 0;
        }
        else {
            d2 = *(data2 + i);
        }
        error += (d1 - d2)*(d1 - d2);
        avg += (d1 + d2);
#ifdef DEBUG
    printf("%10.5f - %10.5f --- %10.5f --- %10.5f\n", d1, d2, (d1 - d2)*(d1 - d2), (d1 + d2));
#endif
    }
    avg /= 2*size;
    v = sqrt(error/(2*size*avg*avg));
#ifdef DEBUG
    printf("------------------------------------------------------\n");
    printf("Error =  %.5f; Promedio =  %.5f\n", error, avg);
    printf("v = %.5f\n",v);
#endif
    if (0 >= 1 - v) {
        max = 0;
    }
    else {
        max = 1-v;
    }
    return max;
}

float precision(float *data1, int size) {
    float sum = 0, d1;
    float avg = 0;
    float v;
    for (int i = 0; i < size; i++) {
        // Data 1
        if(isnan(*(data1 + i))) {
            d1 = 0;
        }
        else {
            d1 = *(data1 + i);
        }
        avg += d1;
#ifdef DEBUG
    printf("%10.5f - %10.5f --- %10.5f --- %10.5f\n", d1, d2, (d1 - d2)*(d1 - d2), (d1 + d2));
#endif
    }
    avg /= size;

    for (int i = 0; i < size; i++) {
        if(isnan(*(data1 + i))) {
            d1 = 0;
        }
        else {
            d1 = *(data1 + i);
        }
        sum += (d1 - avg)*(d1 - avg);
    }

    sum /= size -1;
    v = sqrt(sum)/avg;

    return (1 - v);
}

float accuracy(float *data1, float data2, int size) {
    float d1 = 0;
    float avg = 0;
    float v;
    float max;

    avg = calculateMean(data1, size);
    // for (int i = 0; i < size; i++) {
    //     // Data 1
    //     if(isnan(*(data1 + i))) {
    //         d1 = 0;
    //     }
    //     else {
    //         d1 = *(data1 + i);
    //     }
    //     avg += d1;

    // }

    // avg /= size;

    v = fabs(avg - data2)/data2;

    if (0 >= 1 - v) {
        max = 0;
    }
    else {
        max = 1-v;
    }
    return max;
}

// Pearson Correlation


float PearsonCorrelation(float *list1, float *list2, int size) {

    float mean1 = calculateMean(list1, size);
    float mean2 = calculateMean(list2, size);

    float numerator = 0.0;
    float denominator1 = 0.0;
    float denominator2 = 0.0;

    for (size_t i = 0; i < size; ++i) {
        if(isnan(*(list1 + i)) || isnan(*(list2 + i))) {
            numerator += 0;
        }
        else {
            numerator += (list1[i] - mean1) * (list2[i] - mean2);
            denominator1 += pow(list1[i] - mean1, 2);
            denominator2 += pow(list2[i] - mean2, 2);
            //printf("%10.5f - %10.5f\n",list1[i], list2[i]);
        }

    }

    if (denominator1 == 0 || denominator2 == 0) {
        fprintf(stderr, "Error: One of the lists has zero standard deviation. Pearson's correlation is not defined in this case.\n");
        return 0.0;
    }

    //printf("%10.5f - %10.5f --- %10.5f --- %10.5f --- %10.5f\n", mean1, mean2, numerator, denominator1, denominator2);

    return numerator / sqrt(denominator1 * denominator2);
}

float* plausability(float p_com_df, float p_com_nova, float p_df, float p_nova, float a_df, float a_nova, float *data1, float *data2, int size){
    float sum_df = 0, sum_nova;

    if (p_com_df > 0.75){
        sum_df += (p_com_df - 0.75)/(1 - 0.75);
    }else{
        sum_df += 0;
    }

    if (p_com_nova > 0.75){
        sum_nova += (p_com_nova - 0.75)/(1 - 0.75);
    }else{
        sum_nova += 0;
    }

    if (p_df > 0.9){
        sum_df += (p_df - 0.9)/(1 - 0.9);
    }else{
        sum_df += 0;
    }

    if (p_nova > 0.9){
        sum_nova += (p_nova - 0.9)/(1 - 0.9);
    }else{
        sum_nova += 0;
    }

    if (a_df > 0.9){
        sum_df += (a_df - 0.9)/(1 - 0.9);
    }else{
        sum_df += 0;
    }

    if (a_nova > 0.9){
        sum_nova += (a_nova - 0.9)/(1 - 0.9);
    }else{
        sum_nova += 0;
    }

    float a = sum_df/(sum_df + sum_nova);
    float b = 1 - a;

    float* fusioned = (float*)malloc(size * sizeof(float));
    
    for (int i = 0; i < size; i++) {
        fusioned[i] = (data1[i] * a) + (data2[i] * b);
    }

    return fusioned; 

}

float DQ_Index(float *data, float uncer, float concor, float ref, int listSize){


    float comp = completeness(data, listSize);
    float prec = precision(data, listSize);
    float acc = accuracy(data, ref, listSize);

    
    return (0.4*acc) + (0.08*prec) + (0.18*uncer) + (0.16*comp) + (0.18*concor);
}

