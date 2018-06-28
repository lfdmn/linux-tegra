#include "../lucam.h"



static const unsigned char model_data_029a[] = {
1, 0 , 0 ,0 ,60,0,0,0,
1,0,
234,18,35,251,242,1,26,255,24,18,206,254,7,4,159,249,89,18,
2,0,
127,18,186,251,199,1,247,250,159,20,105,0,62,4,202,249,247,17,
5,0,
167,16,217,252,127,2,50,2,22,15,184,254,134,0,165,0,212,14,
0,0,0,0};



struct _ModelDta2 ModelDtaList[] = 
{

   // Lm130
   {
      0x029a,
      0xffff,
      model_data_029a,
      sizeof(model_data_029a),
   },
};

int ModelDtaListCount = sizeof(ModelDtaList) / sizeof(ModelDtaList[0]);


