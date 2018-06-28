#include "../lucam.h"



static const unsigned char model_data_028c[] = {
1, 0 , 0 ,0 ,60,0,0,0,
1,0,
162,19,84,249,9,3,56,0,220,17,235,253,150,1,64,251,42,19,
2,0,
137,19,12,248,107,4,32,254,8,17,215,0,222,1,132,252,158,17,
5,0,
251,27,136,240,124,3,1,255,132,19,123,253,1,1,56,252,199,18,
0,0,0,0};


struct _ModelDta2 ModelDtaList[] = 
{

   // Lm070
   {
      0x028c,
      0xffff,
      model_data_028c,
      sizeof(model_data_028c),
   },
};

int ModelDtaListCount = sizeof(ModelDtaList) / sizeof(ModelDtaList[0]);


