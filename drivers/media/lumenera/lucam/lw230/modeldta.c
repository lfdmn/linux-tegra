#include "../lucam.h"



static const unsigned char model_data_0180[] = {
1, 0 , 0 ,0 ,80,0,0,0,
1,0,
167,19,91,252,255,255,123,252,226,23,163,251,184,0,12,249,60,22,
2,0,
1,20,197,249,58,2,236,251,81,22,195,253,100,9,1,243,155,19,
3,0,
6,30,112,239,140,2,149,255,242,18,124,253,191,247,99,3,222,20,
5,0,
128,20,234,243,149,7,63,255,36,20,158,252,56,2,45,248,154,21,
0,0,0,0};

//int model_data_size = sizeof(model_data_0180);

struct _ModelDta2 ModelDtaList[] = 
{

   // Lw230
   {
      0x0180,
      0xffff,
      model_data_0180,
      sizeof(model_data_0180),
   },
};

int ModelDtaListCount = sizeof(ModelDtaList) / sizeof(ModelDtaList[0]);


