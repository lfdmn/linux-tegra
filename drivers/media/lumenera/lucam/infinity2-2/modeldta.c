#include "../lucam.h"



static const unsigned char model_data_01a7[] = {
1, 0 , 0 ,0 ,60,0,0,0,
1,0,
167,19,91,252,255,255,123,252,226,23,163,251,184,0,12,249,60,22,
2,0,
1,20,197,249,58,2,236,251,81,22,195,253,100,9,1,243,155,19,
5,0,
128,20,234,243,149,7,63,255,36,20,158,252,56,2,45,248,154,21,
0,0,0,0};


struct _ModelDta2 ModelDtaList[] = 
{

   // Infinity2-2
   {
      0x01a7,
      0xffff,
      model_data_01a7,
      sizeof(model_data_01a7),
   },
};

int ModelDtaListCount = sizeof(ModelDtaList) / sizeof(ModelDtaList[0]);


