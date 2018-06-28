#include "../lucam.h"



static const unsigned char model_data_01cd[] = {
1, 0 , 0 ,0 ,80,0,0,0,
1,0,
56,29,22,251,178,247,72,3,95,17,89,251,247,252,133,252,133,22,
2,0,
129,42,180,233,204,251,140,3,85,17,30,251,82,10,233,235,196,25,
3,0,
85,17,90,5,81,249,16,4,66,16,174,251,53,3,224,248,235,19,
5,0,
248,25,182,0,82,245,48,4,50,21,158,246,160,0,196,0,154,14,
0,0,0,0};



struct _ModelDta2 ModelDtaList[] = 
{

   // Lw290
   {
      0x01cd,
      0xffff,
      model_data_01cd,
      sizeof(model_data_01cd),
   },
};

int ModelDtaListCount = sizeof(ModelDtaList) / sizeof(ModelDtaList[0]);


