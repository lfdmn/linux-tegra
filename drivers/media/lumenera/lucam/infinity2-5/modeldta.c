#include "../lucam.h"



static const unsigned char model_data_04ae[] = {
1, 0 , 0 ,0 ,80,0,0,0,
1,0,
9,22,0,251,251,254,242,252,240,24,34,250,170,253,152,253,192,20,
2,0,
95,23,15,248,146,0,81,251,46,25,129,251,133,252,160,250,220,24,
3,0,
123,24,245,248,144,254,154,252,55,26,47,249,102,253,212,251,198,22,
5,0,
95,23,15,248,146,0,81,251,46,25,129,251,133,252,160,250,220,24,
0,0,0,0};


struct _ModelDta2 ModelDtaList[] = 
{

   // Infinity2-5
   {
      0x04ae,
      0xffff,
      model_data_04ae,
      sizeof(model_data_04ae),
   },
};

int ModelDtaListCount = sizeof(ModelDtaList) / sizeof(ModelDtaList[0]);


