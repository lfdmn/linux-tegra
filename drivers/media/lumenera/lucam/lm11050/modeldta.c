#include "../lucam.h"



static const unsigned char model_data_02c8[] = {
1, 0 , 0 ,0 ,80,0,0,0,
1,0,
51,17,194,254,10,0,128,1,67,16,61,254,67,255,104,2,85,14,
2,0,
163,18,43,253,50,0,6,253,73,19,178,255,44,251,238,1,229,18,
4,0,
135,22,234,247,143,1,195,252,137,20,181,254,158,252,27,248,71,27,
5,0,
2,21,68,251,187,255,185,2,226,19,101,249,230,254,159,0,123,16,

   2, 0 , 0 ,0 ,4,0,0,0,
   0x04,0x00,0x00,0x00,

0,0,0,0};

struct _ModelDta2 ModelDtaList[] = 
{

   // Lm11050
   {
      0x02c8,
      0xffff,
      model_data_02c8,
      sizeof(model_data_02c8)/sizeof(model_data_02c8[0]),
   },
};

int ModelDtaListCount = sizeof(ModelDtaList) / sizeof(ModelDtaList[0]);


