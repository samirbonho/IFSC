/*
 * filters.c
 *
 *  Created on: Nov 6, 2023
 *      Author: samir
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>


uint16_t SmoothData(uint16_t *SensorArray, uint8_t Samples)
{
//This function takes data from an array, removes the highest and lowest 30% of the data and then returns the average of the remaining values first the data is sorted highest to lowest

    uint16_t top = (int)(0.7 * Samples) + 1; // Values used to calculate the top 30% of samples
    uint16_t bottom = (int) (0.3 * Samples); // Values used to calculate the bottom 30% of samples
    printf("%d, %d",top,bottom);
    uint32_t total;                 //used in smoothing data
    uint16_t temporaryData;
    uint32_t sorted[Samples];         // array used to help sort the sensor data
    uint8_t done;
    uint16_t AveragedData;
    uint8_t j=0;

    for (j = 0; j < Samples; j++)
    {     // transfer data array into anther array for sorting and averaging
        sorted[j] = SensorArray[j];
    }
    done = 0;                // flag to know when we're done sorting
    while (done != 1)
    {        //  sorts numbers from lowest to highest
        done = 1;
        for (j = 0; j < (Samples - 1); j++)
        {
            if (sorted[j] > sorted[j + 1])
            {     // numbers are out of order - swap
                temporaryData = sorted[j + 1];
                sorted[j + 1] = sorted[j];
                sorted[j] = temporaryData;
                done = 0;
                //delayMicroseconds(10);   // small delay between each cycle to avoid potential cpu errors
            }
        }
    }
    total = 0;
    for (j = bottom; j < top; j++)
    {   // only select the middle sets of values
        total += sorted[j];             // Add them up
    }
    //AveragedData = (float) total / (top - bottom); //  That is the sum of these values divided by number of samples
    AveragedData = (int) (total / (top - bottom));
    //delay(1);   // small delay to avoid potential cpu errors
    return AveragedData;    // Returns the average of remaining values.
}


