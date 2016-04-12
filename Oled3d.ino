#define OLED_DC 9
#define OLED_CS 12
#define OLED_RESET 10
#define POTENTIOMETER A0

#include <SSD1306.h>
#include <SPI.h>
#include "MyObj.h"
SSD1306 oled(OLED_DC, OLED_RESET);

// cube
/*static MyVertex mp[] ={{ -16, -16,-16}, \
                      {16, -16, -16}, \
                      {16, 16, -16}, \
                      {-16, 16, -16}, \
                      {-16, -16,16}, \
                      {16, -16, 16}, \
                      {16, 16, 16}, \
                      {-16, 16, 16}
};
static MyEdge me[] ={{0, 1}, \
                    {1, 2}, \
                    {2, 3}, \
                    {3, 0}, \
                    {4, 5}, \
                    {5, 6}, \
                    {6, 7}, \
                    {7, 4}, \
                    {0, 4}, \
                    {1, 5}, \
                    {2, 6}, \
                    {3, 7}
};*/
// paper plane
/*static MyVertex mp[] ={{-20,0,-20}, \
                      {0,0,20}, \
                      {20,0,-20}, \
                      {0,0,-20}, \
                      {0,10,-20}
};
static MyEdge me[] ={{0, 1}, \
                    {1, 2}, \
                    {2, 3}, \
                    {3, 4}, \
                    {4, 1}, \
                    {1, 3}, \
                    {3, 0}
};*/

// tank
static MyVertex mp[] ={
              {15,10,16}, \
              {15,10,-21}, \
              {-15,10,-21}, \
              {-15,10,16}, \
              {15,0,23}, \
              {15,0,-27}, \
              {-15,0,-27}, \
              {-15,0,23}, \

              {12,0,9}, \
              {-12,0,9}, \
              {-12,0,-18}, \
              {12,0,-18}, \
              {9,-6,6}, \
              {-9,-6,6}, \
              {-9,-6,-15}, \
              {9,-6,-15}, \

              {0,-3,7.5}, \
              {0,-3,38}, \
              {-3,-3,38}, \
              {3,-3,38}
};
MyEdge me[] ={
              {0, 1}, \
                {1, 2}, \
              {2, 3}, \
                {3, 0}, \
              {4, 5}, \
              {5, 6}, \
              {6, 7}, \
              {7, 4}, \
              {0, 4}, \
              {1, 5}, \
              {2, 6}, \
              {3, 7}, \

              {8, 9}, \
              {9, 10}, \
              {10, 11}, \
              {11, 8}, \
              {12, 13}, \
              {13, 14}, \
              {14, 15}, \
              {15, 12}, \
              {8, 12}, \
              {9, 13}, \
              {10, 14}, \
              {11, 15}, \

              {16, 17}, \
              {17, 18}, \
              {17, 19}
};

MyObject  obj={0, 0, mp, me,{1,0,0,0},{0,0,0}};

void setup()   {
  Serial.begin(9600);
  SPI.setClockDivider(0);
  SPI.begin();

  pinMode(POTENTIOMETER, INPUT_PULLUP);
  //correct vertex num and edge num

  obj.numv=sizeof(mp)/sizeof(MyVertex);
  obj.nume=sizeof(me)/sizeof(MyEdge);

  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
  oled.display(); // show splashscreen
  delay(1000);
  //oled.ssd1306_command(SSD1306_INVERTDISPLAY); //图像反色，注释掉则为黑底白图
  oled.clear();   // clears the screen and buffer
  moveObject(obj,0,0,70);
  renderObject(obj);
  oled.display();
}

static float qdelta[4]={0.999847695f,0,0.0174524f,0};
static float qview[4]={0.25881904510252076234889883762405f,0.9659258262890682867497431997289f,0,0};
static float qtemp[4];

#define near 1.0
#define far 100.0
#define right 1.8
#define top 1.8

static float proj[8]={
    near/right,    0,         0,                    0, \
    0,         near/top,      0,                    0
//    0,             0,   -(far+near)/(far-near), -2*far*near/(far-near),
//    0,             0,         -1,                   0
};
void loop()
{
  rotateObject(obj,qdelta);
  oled.clear();   // clears the screen and buffer
  renderObject(obj);
  oled.display();
}


void moveObject(MyObject &mo, float x, float y, float z) {
  mo.offset[0]=x;
  mo.offset[1]=y;
  mo.offset[2]=z;
}

void rotateObject(MyObject &mo, float* q) {
  qproduct(q,obj.quat,qtemp);
  mo.quat[0]=qtemp[0];
  mo.quat[1]=qtemp[1];
  mo.quat[2]=qtemp[2];
  mo.quat[3]=qtemp[3];
  qnormalized(qtemp);
}

void renderObject(MyObject &mo) {
  static int8_t horizon[3]={0,0,10000};
  static float accang=0;
  int16_t in = analogRead(A0)-512;
  float ang =(in-512)/512.0f*PI;
  accang=0.9*accang+0.1*ang;
  qview[0]=cos(accang/2);
  qview[1]=sin(accang/2);
  MyVertex* mv=new MyVertex[mo.numv];

  {
    float vhtemp[3];
    iqRot(qview,horizon,vhtemp);
    MatMulVect(proj, vhtemp, mv[0].location);
    oled.drawline(0, mv[0].location[1]+32, 128, mv[0].location[1]+32, WHITE);
  }

  qproduct(qview,mo.quat,qtemp);

  for (int i = 0; i < mo.numv; i++) {
    float vtemp[3];
    iqRot(qtemp,mo.v[i].location,vtemp);
    vtemp[0] += mo.offset[0];
    vtemp[1] += mo.offset[1];
    vtemp[2] += mo.offset[2];

    MatMulVect(proj, vtemp, mv[i].location);
  }

  
  for (int i = 0; i < mo.nume; i++) {
    int8_t p1 = mo.e[i].connection[0];
    int8_t p2 = mo.e[i].connection[1];
    oled.drawline(mv[p1].location[0]+64, mv[p1].location[1]+32, mv[p2].location[0]+64, mv[p2].location[1]+32, WHITE);
  }
  
  delete mv;
}

float iqRot(float q[],int8_t v[],float result[]){
  float prod[4];
  prod[0] =  - q[1] * v[0] - q[2] * v[1] - q[3] * v[2];
  prod[1] = q[0] * v[0] + q[2] * v[2] - q[3] * v[1];
  prod[2] = q[0] * v[1] - q[1] * v[2] + q[3] * v[0];
  prod[3] = q[0] * v[2] + q[1] * v[1] - q[2] * v[0];
  
  result[0] = -prod[0] * q[1] + prod[1] * q[0] - prod[2] * q[3] + prod[3] * q[2];
  result[1] = -prod[0] * q[2] + prod[1] * q[3] + prod[2] * q[0] - prod[3] * q[1];
  result[2] = -prod[0] * q[3] - prod[1] * q[2] + prod[2] * q[1] + prod[3] * q[0];
}

void qproduct(const float* p, const float* q, float* qr) {
  qr[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
  qr[1] = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2];
  qr[2] = p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1];
  qr[3] = p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0];
}

void qnormalized(float* q) {
  float invnorm;
  invnorm = fastinvsqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (invnorm < 100000000) {
    q[0] *= invnorm;
    q[1] *= invnorm;
    q[2] *= invnorm;
    q[3] *= invnorm;
  } else {
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
  }
}
// fast invsqrt by John Carmark
float fastinvsqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

float MatMulVect(float M[], float V[], int8_t result[]){
  for(uint8_t i=0; i<2; i++){
     result[i] = (M[i*4]*V[0]+M[i*4+1]*V[1]+M[i*4+2]*V[2]+M[i*4+3])/-V[2]*128;
  }
}

//作者：微风森林
//author: SylvanYZY