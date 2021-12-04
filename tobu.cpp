#include "tobu.hpp"
#include "pwm_uart.hpp"
#include "ekf_sensor.hpp"
/* -----------------------------------------------------------------------------------------------------------------------*/
//マルチコア設定
semaphore_t sem;

float ax,ay,az,wp,wq,wr,mx,my,mz,wqa=0.0,wpa=0.0,wra=0.0,Wqa,Wpa,Wra; 
float Mn,Md;
Matrix<float, 7 ,1> xp = MatrixXf::Zero(7,1);
Matrix<float, 7 ,1> xe = MatrixXf::Zero(7,1);
Matrix<float, 7 ,1> x_sim = MatrixXf::Zero(7,1);
Matrix<float, 7 ,7> P = MatrixXf::Identity(7,7);
Matrix<float, 6 ,1> z = MatrixXf::Zero(6,1);
Matrix<float, 3, 1> omega_m = MatrixXf::Zero(3, 1);
Matrix<float, 6, 6> Q = MatrixXf::Identity(6, 6)*1;
Matrix<float, 6, 6> R = MatrixXf::Identity(6, 6)*1;
Matrix<float, 7 ,6> G;
Matrix<float, 3 ,1> beta;
volatile float kalman_time=0.0;
float spsi,sphi,st;

float dt=0.01;
short Hzcount=1;
float duty_rr,duty_rl,duty_fl,duty_fr; 
float Duty_rr,Duty_rl,Duty_fl,Duty_fr; 
float ref_e,ref_r,ref_a,err_e=0.0,err_a=0.0,err_r=0.0;
float sk_e,sk_a,sk_r;
float olderr_e,olderr_a,olderr_r;
float dk_a,dk_e,dk_r;
float se,sa,sr;
float Ti=10000;
float Td=0.0;
float h=0.01;
float mn,md,me;
volatile float dmx,dmy,dmz,dxx,dxy,dxz;
float kpe=0.50,kpa=0.50,kpr=0.50;
const uint16_t Logdatanum=48000;
float logdata[Logdatanum]={0.0};
const uint LED_PIN = 25;
float phi,theta,psi;
float psiav,phiav,thetaav;
float PSI,PHI,THETA;
char sbuf[256];
/*----------------------------------------------------------------------------------------------------------------------------------------------*/

void kalman(void){
   
    float ref_t,ref_phi,ref_psi,err_psi=0.0,err_phi=0.0,err_t=0.0;
    float sk_phi,sk_t,sk_psi;
    float olderr2_phi=0.0,olderr3_phi=0.0,olderr3_t=0.0,olderr2_t=0.0;
    float olderr_phi,olderr_psi,olderr_t;
    float dk_phi,dk_psi,dk_t;
    float kppsi=0.0,kpphi=0.0,kpt=0.0;
    const uint8_t DATANUM=28;
    uint32_t logcount=0;
    uint32_t printcount=0;
    float data2MID,data4MID;
    float T=0.0;
    gpio_init(LED_PIN);                       //gpioを使えるようにする
    gpio_set_dir(LED_PIN, GPIO_OUT);
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
  while(1)
  {
    sem_acquire_blocking(&sem);
    sem_reset(&sem, 0);
    //printf("%f\n",time);
    ref_t=(Data2-data2MID)*0.523598775598299;
    ref_phi=(Data4-data4MID)*0.52398775598299;
    ref_psi=Data1;
    dt=0.01;
    omega_m <<wp,wq,wr;
    z       <<ax,ay,az,mx,my,mz;//ここに入れる
    //--Begin Extended Kalman Filter--
    ekf(xp, xe, P, z, omega_m, Q, R, G*dt, beta, dt);
    kalman_time = kalman_time + 0.01;
   
    phi=Phl(xe);
    theta=Theta(xe);
    psi=Psi(xe);

    PSI=psi-psiav;
    PHI=phi-phiav;
    THETA=theta-thetaav;


    if(Data3>=0.30){

      //phi
      olderr3_phi=olderr2_phi;
      olderr2_phi=olderr_phi;
      olderr_phi=err_phi;
      err_phi=(ref_phi-PHI);
      if (sk_phi<=3000){
        sk_phi=sk_phi+olderr_phi;
      }
      else if(-3000<=sk_phi){
        sk_phi=sk_phi+olderr_phi;
      }
      dk_phi=(err_phi-olderr3_phi)*400;
      sphi=kpphi*(err_phi+1/Ti*sk_phi+Td*dk_phi);
      
      //theta
      olderr3_t=olderr2_t;
      olderr2_t=olderr_t;
      olderr_t=err_t;
      err_t=(ref_t-THETA);
      if (sk_t<=3000){
        sk_t=sk_t+olderr_t;
      }
      else if(-3000<=sk_t){
        sk_t=sk_t+olderr_t;
      }
      dk_t=(err_t-olderr3_t)*400;
      st=kpt*(err_t+1/Ti*sk_t+Td*dk_t);
    }
    else{
      sk_phi=0;
      sk_t=0;
      data2MID=Data2;
      data4MID=Data4;
    }    
//スティック上
  if(Data6<0.0){
    if(Data3>=0.0){
      if(logcount<47500){
        gpio_put(LED_PIN, 1);
        logdata[logcount++]=xe(0,0);
        logdata[logcount++]=xe(1,0);
        logdata[logcount++]=xe(2,0);
        logdata[logcount++]=xe(3,0);
        logdata[logcount++]=xe(4,0);
        logdata[logcount++]=xe(5,0);
        logdata[logcount++]=xe(6,0);
        logdata[logcount++]=wp;
        logdata[logcount++]=wq;
        logdata[logcount++]=wr;
        logdata[logcount++]=ax;
        logdata[logcount++]=ay;
        logdata[logcount++]=az;
        logdata[logcount++]=mx;
        logdata[logcount++]=my;
        logdata[logcount++]=mz;
        logdata[logcount++]=ref_a;
        logdata[logcount++]=ref_e;
        logdata[logcount++]=ref_r;
        logdata[logcount++]=PHI;
        logdata[logcount++]=THETA;
        logdata[logcount++]=PSI;
        logdata[logcount++]=ref_phi;
        logdata[logcount++]=ref_t;
        logdata[logcount++]=ref_psi;
        logdata[logcount++]=sa;
        logdata[logcount++]=se;
        logdata[logcount++]=sr;


    }
      else{
        gpio_put(LED_PIN,0);
      }
    }
  }
  
  //スティック下
  else if(Data6>0.0){
    if(Data3<=0.3){
      gpio_put(LED_PIN, 0);
      sleep_ms(500);
      gpio_put(LED_PIN,1);
      sleep_ms(500);
      while(logcount>=printcount){
          for (uint8_t i=0;i<DATANUM;i++){
            if(i==0){
              printf("%8.2f ",T);
              T=T+0.01;

            }
             sprintf(sbuf,"%12.5f",logdata[printcount+i]);
             printf("%s",sbuf);
          }

          printf("\n");
          printcount=printcount+DATANUM;
    }
    }
  }
  else if(Data3==0){
    printcount=0.0;
    T=0.0;
  }
//スティック下
  if(Data5>0.0){ 
   gpio_put(LED_PIN, 1);
   logcount=0;
   printcount=0;
   T=0.0;
}
}
}
/*---------------------------------------------------------------------------------------------------------------------------------------------------------*/
void MAINLOOP(void)
{

  float mx1,my1,mz1;
  const uint LED_PIN = 25;
  float olderr2_a=0.0,olderr3_a=0.0,olderr2_r=0.0,olderr3_r=0.0,olderr2_e=0.0,olderr3_e=0.0;
  //LED_PIN=0  
  gpio_init(25);                       //gpioを使えるようにする
  gpio_set_dir(25, GPIO_OUT);
  //e エレベータ a エルロン r ラダー t スロットル
  pwm_clear_irq(2);
  imu_mag_data_read();
  ax=   -acceleration_mg[0]*0.001*GRAV;
  ay=   -acceleration_mg[1]*0.001*GRAV;
  az=    acceleration_mg[2]*0.001*GRAV;
  wp=    angular_rate_mdps[0]*0.001*0.017453292;
  wq=    angular_rate_mdps[1]*0.001*0.017453292;
  wr=   -angular_rate_mdps[2]*0.001*0.017453292;
  dmx=  -(magnetic_field_mgauss[0]);
  dmy=   (magnetic_field_mgauss[1]);
  dmz=  -(magnetic_field_mgauss[2]);

  //回転行列
  const float rot[9]={0.65330968, 0.75327755, -0.07589064,
                     -0.75666134, 0.65302622, -0.03194321,
                      0.02549647, 0.07829232,  0.99660436};
  //中心座標
  const float center[3]={122.37559195017053, 149.0184454603531, -138.99116060635413};
  //拡大係数
  const float zoom[3]={0.003077277151877191, 0.0031893151610213463, 0.0033832794976645804};

  //回転・平行移動・拡大
  mx1 = zoom[0]*( rot[0]*dmx +rot[1]*dmy +rot[2]*dmz -center[0]);
  my1 = zoom[1]*( rot[3]*dmx +rot[4]*dmy +rot[5]*dmz -center[1]);
  mz1 = zoom[2]*( rot[6]*dmx +rot[7]*dmy +rot[8]*dmz -center[2]);
  //逆回転
  mx = rot[0]*mx1 +rot[3]*my1 +rot[6]*mz1;
  my = rot[1]*mx1 +rot[4]*my1 +rot[7]*mz1;
  mz = rot[2]*mx1 +rot[5]*my1 +rot[8]*mz1; 
  float mag_norm=sqrt(mx*mx +my*my +mz*mz);
  mx/=mag_norm;
  my/=mag_norm;
  mz/=mag_norm;


  if(Hzcount<=3){
    Hzcount=Hzcount+1;
  }
  else{
    Hzcount=1;
    sem_release(&sem);
  }

#if 1    
    //姿勢安定化
    //最大角速度 e,a 6π,r 2π
    //最大角度30°
  ref_e=st;
  ref_a=sphi;
  ref_r=Data1*6.283185307;

  
    //エレベータq
  
  olderr3_e=olderr2_e;
  olderr2_e=olderr_e;
  olderr_e=err_e;
  err_e=(ref_e - (wq-Wqa));
  if (sk_e<=30000){
    sk_e=sk_e+err_e;
  }
  else if(-30000<=sk_e){
    sk_e=sk_e+err_e;
  }
  dk_e=(err_e-olderr3_e)*400;
  se=kpe*(err_e);//+1/Ti*sk_e+Td*dk_e);

  //エルロンp

  olderr3_a=olderr2_a;
  olderr2_a=olderr_a;
  olderr_a=err_a;
  err_a=(ref_a - (wp-Wpa));
  if (sk_a<=30000){
    sk_a=sk_a+err_a;
  }
  else if(-30000<=sk_a){
    sk_a=sk_a+err_a;
  }
  dk_a=(err_a-olderr3_a)*400;
  sa=kpa*(err_a);//+1/Ti*sk_a+Td*dk_a);

  //ラダーr

  
  olderr3_r=olderr2_r;
  olderr2_r=olderr_r;
  olderr_r=err_r;
  err_r=(ref_r - (wr-Wra));
  if (sk_r<=30000){
    sk_r=sk_r+err_r;
  }
  else if(-30000<=sk_r){
    sk_r=sk_r+err_r;
  }
  dk_r=(err_r-olderr3_r)*400;
  sr=kpr*(err_r);//+1/Ti*sk_r+Td*dk_r);

  if(kalman_time>=25.0){
#if 1
   Duty_fr=Data3+(se-sa+sr)*0.25;
    Duty_fl=Data3+(se+sa-sr)*0.25;
    Duty_rr=Data3+(-se-sa-sr)*0.25;         
    Duty_rl=Data3+(-se+sa+sr)*0.25;
  }
#endif
#endif
#if 0   
    Duty_fr=Data3;
    Duty_fl=Data3;
    Duty_rr=Data3;         
    Duty_rl=Data3;
  else{
    Duty_fr=Data3*0;
    Duty_fl=Data3*0;
    Duty_rr=Data3*0;         
    Duty_rl=Data3*0;
  }
#endif
  tight_loop_contents();




  duty_rr=(float)(DUTYMAX-DUTYMIN)*Duty_rr+DUTYMIN;
  duty_fr=(float)(DUTYMAX-DUTYMIN)*Duty_fr+DUTYMIN;
  duty_rl=(float)(DUTYMAX-DUTYMIN)*Duty_rl+DUTYMIN;
  duty_fl=(float)(DUTYMAX-DUTYMIN)*Duty_fl+DUTYMIN;




  if (duty_rr>DUTYMAX-70.0)duty_rr=DUTYMAX-70.0;
  if (duty_rr<DUTYMIN+30.0)duty_rr=DUTYMIN+30.0;
  if (duty_fr>DUTYMAX-50.0)duty_fr=DUTYMAX-50.0;
  if (duty_fr<DUTYMIN+15.0)duty_fr=DUTYMIN+15.0;
  if (duty_rl>DUTYMAX-50.0)duty_rl=DUTYMAX-50.0;
  if (duty_rl<DUTYMIN+15.0)duty_rl=DUTYMIN+15.0;
  if (duty_fl>DUTYMAX-50.0)duty_fl=DUTYMAX-50.0;
  if (duty_fl<DUTYMIN+15.0)duty_fl=DUTYMIN+15.0;

  if(Data3<0.05)
  {
    pwm_set_chan_level(slice_num[0], PWM_CHAN_A, DUTYMIN);
    pwm_set_chan_level(slice_num[0], PWM_CHAN_B, DUTYMIN);
    pwm_set_chan_level(slice_num[1], PWM_CHAN_A, DUTYMIN);
    pwm_set_chan_level(slice_num[1], PWM_CHAN_B, DUTYMIN);
    sk_r=0.0;
    sk_a=0.0;
    sk_e=0.0;
    err_r=0.0;
    err_a=0.0;
    err_e=0.0;


  }
  else
  {
    pwm_set_chan_level(slice_num[0], PWM_CHAN_A, duty_rr);
    pwm_set_chan_level(slice_num[0], PWM_CHAN_B, duty_fr);
    pwm_set_chan_level(slice_num[1], PWM_CHAN_A, duty_rl);
    pwm_set_chan_level(slice_num[1], PWM_CHAN_B, duty_fl);
  }
  //printf("%04f %04f %04f %04f %04f %04f \n",Olddata[0],Olddata[1],Olddata[2],Olddata[3],Olddata[4],Olddata[5]);    
  //e_time=time_us_64();
  //d_time=e_time-s_time;
  //printf("右前%04f   左前%04f   右後ろ%04f   左後ろ%04f\n",duty_fr,duty_fl,duty_rr,duty_rl);
  //printf("%04f \n",t);
  //t=t+1;
  
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------------------*/
int main(void)
{
  float w,f=1;
  float Thetaplas=0.0,Psiplas=0.0,Phiplas=0.0;
  const uint LED_PIN = 25;        //LED_PIN=0  
  gpio_init(25);                  //gpioを使えるようにする
  gpio_set_dir(25, GPIO_OUT);
  
  xe << 1.00, 0.0, 0.0, 0.0,0.0,0.0, 0.0;
  xp =xe;

  Q <<  0.0363, 0.0   , 0.0     , 0.0   , 0.0   , 0.0,
        0.0   , 0.0298, 0.0     , 0.0   , 0.0   , 0.0,
        0.0   , 0.0   , 0.000513, 0.0   , 0.0   , 0.0,
        0.0   , 0.0   , 0.0     , 0.5e-5, 0.0   , 0.0,
        0.0   , 0.0   , 0.0     , 0.0   , 0.5e-5, 0.0,
        0.0   , 0.0   , 0.0     , 0.0   , 0.0   , 1.0;

  R <<  29.1   , 0.0    , 0.0    , 0.0     , 0.0     , 0.0,
        0.0    , 29.9   , 0.0    , 0.0     , 0.0     , 0.0,
        0.0    , 0.0    , 32.2   , 0.0     , 0.0     , 0.0,
        0.0    , 0.0    , 0.0    , 0.000416, 0.0     , 0.0,
        0.0    , 0.0    , 0.0    , 0.0     , 0.000372, 0.0,
        0.0    , 0.0    , 0.0    , 0.0     , 0.0     , 0.000467;
          
  G <<   1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 
        -1.0, 1.0,-1.0, 0.0, 0.0, 0.0, 
        -1.0,-1.0, 1.0, 0.0, 0.0, 0.0, 
         1.0,-1.0,-1.0, 0.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  beta << 0.0, 0.0, 0.0;
  
  P <<  1e0,0,0,0,0,0,0,  
        0,1e0,0,0,0,0,0,
        0,0,1e0,0,0,0,0,  
        0,0,0,1e0,0,0,0, 
        0,0,0,0,1e0,0,0,  
        0,0,0,0,0,1e0,0,  
        0,0,0,0,0,0,1e0;


  //gpio_put(LED_PIN, 1);
  stdio_init_all();
  //sleep_ms(1000);
  imu_mag_init();
 // gpio_put(LED_PIN, 0);
  serial_settei();
  gpio_put(LED_PIN, 1);
  

  /* for (i=0;i<waittime;i++)
  {
    printf("#Please wait %d[s] ! \n",waittime-i);
    sleep_ms(1000);
  }
  printf("#Start Kalman Filter\n");
*/
  wpa=0.0;
  wqa=0.0;
  wra=0.0;

    while(f<=400){
      float mx1,my1,mz1;
      imu_mag_data_read();
      wp=    angular_rate_mdps[0]*0.001*0.017453292;
      wq=    angular_rate_mdps[1]*0.001*0.017453292;
      wr=   -angular_rate_mdps[2]*0.001*0.017453292;
      dmx=  -(magnetic_field_mgauss[0]);
      dmy=   (magnetic_field_mgauss[1]);
      dmz=  -(magnetic_field_mgauss[2]);

      //回転行列
      const float rot[9]={0.65330968, 0.75327755, -0.07589064,
        -0.75666134, 0.65302622, -0.03194321,
        0.02549647, 0.07829232,  0.99660436};
      //中心座標
      const float center[3]={122.37559195017053, 149.0184454603531, -138.99116060635413};
      //拡大係数
      const float zoom[3]={0.003077277151877191, 0.0031893151610213463, 0.0033832794976645804};

      //回転・平行移動・拡大
      mx1 = zoom[0]*( rot[0]*dmx +rot[1]*dmy +rot[2]*dmz -center[0]);
      my1 = zoom[1]*( rot[3]*dmx +rot[4]*dmy +rot[5]*dmz -center[1]);
      mz1 = zoom[2]*( rot[6]*dmx +rot[7]*dmy +rot[8]*dmz -center[2]);
      //逆回転
      mx = rot[0]*mx1 +rot[3]*my1 +rot[6]*mz1;
      my = rot[1]*mx1 +rot[4]*my1 +rot[7]*mz1;
      mz = rot[2]*mx1 +rot[5]*my1 +rot[8]*mz1; 
      float mag_norm=sqrt(mx*mx +my*my +mz*mz);
      mx/=mag_norm;
      my/=mag_norm;
      mz/=mag_norm;

      mn=mn+mx;
      md=md+my;
      me=me+mz;
      wqa=wq+wqa;
      wpa=wp+wpa;
      wra=wr+wra;
      f=f+1;
      sleep_us(2500);
      }
  Wqa=wqa/400;
  Wpa=wpa/400;
  Wra=wra/400;
  MN=mn/400;
  MD=md/400;
  ME=me/400;



  //printf("%f,%f\n",Mn,Md);

 // gpio_put(LED_PIN, 1);
  sem_init(&sem, 0, 1);
  multicore_launch_core1(kalman);
  
  
  pwm_settei();
  
  while(kalman_time<20.0);

  f=0.0;
  while(f<4000){
        Phiplas=Phiplas+phi;
        Thetaplas=Thetaplas+theta;
        Psiplas=Psiplas+psi;
        f=f+1;
  }
  phiav=Phiplas/4000;
  thetaav=Thetaplas/4000;
  psiav=Psiplas/4000;
  gpio_put(LED_PIN,1);
 // float old_time=-1.0;
 // char txbuf[256];

  while(1)
  {}
 
}

