#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../mpu9250_simple.c"

static uint8_t bits[8192];
static unsigned nbits, cursor;
static uint32_t bus_us;
void SDA(uint8_t level) { assert(level <= 1); }
void SCK(uint8_t level) { assert(level <= 1); }
uint8_t SDA_Read(void) { assert(cursor < nbits); return bits[cursor++]; }
void Delay_ms(uint32_t ms) { (void)ms; }
void Delay_us(uint32_t us) { bus_us += us; }
static void queue_write(void) { bits[nbits++]=1;bits[nbits++]=0;bits[nbits++]=0;bits[nbits++]=0; }
static void queue_read(const uint8_t *data,unsigned n) {
    unsigned i,j;
    bits[nbits++]=1;bits[nbits++]=0;bits[nbits++]=0;bits[nbits++]=1;bits[nbits++]=0;
    for(i=0;i<n;i++)for(j=0;j<8;j++)bits[nbits++]=(uint8_t)((data[i]>>(7-j))&1);
}
static void initialize(void) {
    unsigned i;uint8_t who=0x71,wia=0x48,asa[3]={128,128,128};
    cursor=nbits=0;queue_read(&who,1);
    for(i=0;i<12;i++)queue_write();
    queue_read(&wia,1);for(i=0;i<3;i++)queue_write();
    queue_read(asa,3);queue_write();queue_write();
    assert(MPU9250_Init());assert(cursor==nbits);
}
static void sensors(float roll,float pitch,float yaw,int16_t acc[3],int16_t mag[3]) {
    float r=roll*DEG_TO_RAD,p=pitch*DEG_TO_RAD,y=yaw*DEG_TO_RAD;
    float sr=sinf(r),cr=cosf(r),sp=sinf(p),cp=cosf(p),sy=sinf(y),cy=cosf(y);
    float mx=0.8f*cy*cp-0.6f*sp;
    float my=0.8f*(cy*sp*sr-sy*cr)+0.6f*cp*sr;
    float mz=0.8f*(cy*sp*cr+sy*sr)+0.6f*cp*cr;
    acc[0]=(int16_t)lroundf(-sp*16384);acc[1]=(int16_t)lroundf(cp*sr*16384);acc[2]=(int16_t)lroundf(cp*cr*16384);
    mag[0]=(int16_t)lroundf(my*10000);mag[1]=(int16_t)lroundf(mx*10000);mag[2]=(int16_t)lroundf(-mz*10000);
}

static float angle_error(float a,float b) {return remainderf(a-b,360.0f);}
static void expect_pose(const float a[3],float r,float p,float y,float tolerance) {
    assert(fabsf(a[0]-p)<tolerance);
    assert(fabsf(angle_error(a[1],r))<tolerance);
    assert(fabsf(angle_error(a[2],y))<tolerance);
    assert(isfinite(a[0]) && isfinite(a[1]) && isfinite(a[2]));
    assert(a[0]>=-90.001f && a[0]<=90.001f);
    assert(a[1]>=-180.001f && a[1]<=180.001f);
    assert(a[2]>=-180.001f && a[2]<=180.001f);
}
static void test_raw(void) {
    uint8_t raw[6]={0x80,0,0x7f,0xff,0xff,0xff};
    uint8_t st1=1,mag_raw[7]={0,0x80,0xff,0x7f,0xff,0xff,0x10};
    int16_t out[3]={123,123,123};
    initialize();
    cursor=nbits=0;bus_us=0;queue_read(raw,6);assert(MPU9250_ReadAccel(out));assert(cursor==nbits);
    assert(out[0]==-32768 && out[1]==32767 && out[2]==-1);assert(bus_us==513);
    cursor=nbits=0;queue_read(raw,6);assert(MPU9250_ReadGyro(out));assert(cursor==nbits);
    assert(out[0]==-32768 && out[1]==32767 && out[2]==-1);
    cursor=nbits=0;queue_read(&st1,1);queue_read(mag_raw,7);assert(MPU9250_ReadMag(out));assert(cursor==nbits);
    assert(out[0]==-32768 && out[1]==32767 && out[2]==-1);
    mag_raw[6]|=8;out[0]=123;cursor=nbits=0;queue_read(&st1,1);queue_read(mag_raw,7);assert(!MPU9250_ReadMag(out) && out[0]==123);
    st1=0;cursor=nbits=0;queue_read(&st1,1);assert(!MPU9250_ReadMag(out) && out[0]==123 && cursor==nbits);
    /* Address NACK and occupied bus must not alter output. */
    cursor=0;nbits=2;bits[0]=1;bits[1]=1;assert(!MPU9250_ReadAccel(out) && out[0]==123 && cursor==nbits);
    cursor=0;nbits=1;bits[0]=0;assert(!MPU9250_ReadGyro(out) && out[0]==123);
    puts("PASS: signed burst reads, ACK failures, magnetic DRDY/ST2 and unchanged outputs");
}
static void test_lifecycle(void) {
    int16_t acc[3],mag[3],gyro[3]={0,0,0},zero[3]={0,0,0};
    float a[3],before[3];uint8_t wrong=0;int i;
    initialize();sensors(20,30,45,acc,mag);
    assert(MPU9250_CalcAngles(acc,gyro,NULL,0,a));expect_pose(a,20,30,0,.04f);
    initialize();assert(MPU9250_CalcAngles(acc,gyro,mag,0,a));expect_pose(a,20,30,45,.04f);
    initialize();sensors(-15,-25,-120,acc,mag);
    assert(MPU9250_CalcAngles(acc,gyro,mag,0,a));expect_pose(a,-15,-25,-120,.04f);
    assert(eint[0]==0 && eint[1]==0 && eint[2]==0);
    memcpy(before,a,sizeof(a));assert(!MPU9250_CalcAngles(acc,gyro,mag,101,a));assert(!memcmp(before,a,sizeof(a)));
    sensors(5,10,110,acc,mag);
    /* Even a legacy caller counting from last success can recover. */
    assert(MPU9250_CalcAngles(acc,gyro,mag,106,a));expect_pose(a,5,10,110,.04f);
    assert(!MPU9250_CalcAngles(acc,gyro,NULL,500,a));
    assert(!MPU9250_CalcAngles(zero,gyro,NULL,5,a));
    assert(MPU9250_CalcAngles(acc,gyro,NULL,5,a));expect_pose(a,5,10,0,.04f);
    memcpy(before,a,sizeof(a));
    assert(!MPU9250_CalcAngles(acc,gyro,mag,0,a));
    assert(!MPU9250_CalcAngles(acc,gyro,mag,-1,a));
    assert(!MPU9250_CalcAngles(acc,gyro,mag,NAN,a));
    assert(!MPU9250_CalcAngles(acc,gyro,mag,INFINITY,a));
    assert(!MPU9250_CalcAngles(NULL,gyro,mag,5,a));
    assert(!MPU9250_CalcAngles(acc,NULL,mag,5,a));
    assert(!MPU9250_CalcAngles(acc,gyro,mag,5,NULL));
    assert(!memcmp(before,a,sizeof(a)));
    assert(MPU9250_CalcAngles(acc,gyro,mag,5,a));
    /* A failed hardware reinitialization must revoke old ready/filter state. */
    cursor=nbits=0;queue_read(&wrong,1);assert(!MPU9250_Init());
    memcpy(before,a,sizeof(a));assert(!MPU9250_CalcAngles(acc,gyro,mag,5,a));assert(!memcmp(before,a,sizeof(a)));
    for(i=0;i<3;i++)assert(eint[i]==0);
    assert(!ready && !quat_ready);
    puts("PASS: optional initial magnetometer, reinitialization, millisecond dt, timeout recovery and invalid inputs");
}
static void test_static_poses(void) {
    const float rolls[]={-170,-45,0,35,170},pitches[]={-60,0,60},yaws[]={-170,-60,0,110,179};
    int ri,pi,yi,k;int16_t acc[3],gyro[3]={0,0,0},mag[3];float a[3];
    for(ri=0;ri<5;ri++)for(pi=0;pi<3;pi++)for(yi=0;yi<5;yi++) {
        initialize();sensors(rolls[ri],pitches[pi],yaws[yi],acc,mag);
        assert(MPU9250_CalcAngles(acc,gyro,mag,0,a));
        expect_pose(a,rolls[ri],pitches[pi],yaws[yi],.05f);
        for(k=0;k<200;k++)assert(MPU9250_CalcAngles(acc,gyro,mag,5,a));
        expect_pose(a,rolls[ri],pitches[pi],yaws[yi],.06f);
    }
    puts("PASS: 75 independent gravity/magnetic-field poses, Pitch/Roll/Yaw ordering and signed yaw");
}
static void test_rotations(void) {
    int axis,sign,i;int16_t acc[3],gyro[3],mag[3];float a[3],qnorm;
    float alpha=5.0f/(5.0f+1000.0f/(2.0f*PI_F*MPU9250_GYRO_LPF_HZ));
    float during=90.0f*.005f*(100.0f-(1-alpha)*(1-powf(1-alpha,100))/alpha);
    for(axis=0;axis<3;axis++)for(sign=-1;sign<=1;sign+=2) {
        initialize();sensors(0,0,0,acc,mag);memset(gyro,0,sizeof(gyro));
        assert(MPU9250_CalcAngles(acc,gyro,mag,0,a));memset(acc,0,sizeof(acc));gyro[axis]=(int16_t)(sign*1476);
        for(i=0;i<100;i++)assert(MPU9250_CalcAngles(acc,gyro,NULL,5,a));
        expect_pose(a,axis==0?sign*during:0,axis==1?sign*during:0,axis==2?sign*during:0,.02f);
        memset(gyro,0,sizeof(gyro));
        for(i=0;i<40;i++)assert(MPU9250_CalcAngles(acc,gyro,NULL,5,a));
        expect_pose(a,axis==0?sign*45:0,axis==1?sign*45:0,axis==2?sign*45:0,.01f);
        qnorm=quat[0]*quat[0]+quat[1]*quat[1]+quat[2]*quat[2]+quat[3]*quat[3];assert(fabsf(qnorm-1)<1e-5f);
    }
    initialize();sensors(0,0,179,acc,mag);memset(gyro,0,sizeof(gyro));
    assert(MPU9250_CalcAngles(acc,gyro,mag,0,a));memset(acc,0,sizeof(acc));gyro[2]=1640;
    for(i=0;i<4;i++)assert(MPU9250_CalcAngles(acc,gyro,NULL,5,a));
    gyro[2]=0;for(i=0;i<40;i++)assert(MPU9250_CalcAngles(acc,gyro,NULL,5,a));
    expect_pose(a,0,0,-179,.02f);
    printf("Gyro LPF step: ideal 45deg, estimate at stop %.4fdeg, settled 45deg\n",during);
    puts("PASS: signed 3-axis gyro integration, LPF phase delay, 5ms scaling, quaternion norm and signed yaw");
}
static void test_nine_axis_lpf(void) {
    int16_t acc0[3]={0,0,16384},gyro0[3]={0,0,0},mag0[3]={0,10000,-5000};
    int16_t acc[3]={1000,-2000,16000},gyro[3]={1000,-2000,3000},mag[3]={1000,9000,-4000};
    float a[3],ag=5/(5+1000/(2*PI_F*MPU9250_GYRO_LPF_HZ));
    float aa=5/(5+1000/(2*PI_F*MPU9250_ACCEL_LPF_HZ));
    float am=10/(10+1000/(2*PI_F*MPU9250_MAG_LPF_HZ));
    float before[3],expected;int i,j;
    initialize();assert(MPU9250_CalcAngles(acc0,gyro0,mag0,0,a));
    for(i=0;i<3;i++) {assert(filtered[0][i]==acc0[i]);assert(filtered[1][i]==gyro0[i]);assert(filtered[2][i]==mag0[i]);}
    assert(MPU9250_CalcAngles(acc,gyro,NULL,5,a));
    for(i=0;i<3;i++) {
        assert(fabsf(filtered[0][i]-(acc0[i]+aa*(acc[i]-acc0[i])))<.002f);
        assert(fabsf(filtered[1][i]-ag*gyro[i])<.002f);
        assert(filtered[2][i]==mag0[i]);
    }
    assert(MPU9250_CalcAngles(acc,gyro,mag,5,a));
    for(i=0;i<3;i++) {assert(fabsf(filtered[2][i]-(mag0[i]+am*(mag[i]-mag0[i])))<.002f);before[i]=filtered[1][i];}
    assert(MPU9250_CalcAngles(acc,gyro0,NULL,10,a));
    for(i=0;i<3;i++) {
        expected=before[i]*(1-10/(10+1000/(2*PI_F*MPU9250_GYRO_LPF_HZ)));
        assert(fabsf(filtered[1][i]-expected)<.002f);
    }
    /* Reject an acceleration shock before it can contaminate the gravity LPF. */
    acc[0]=32767;assert(MPU9250_CalcAngles(acc,gyro0,NULL,5,a));assert(!filter_ready[0]);
    assert(MPU9250_CalcAngles(acc0,gyro0,NULL,5,a));
    for(i=0;i<3;i++)assert(filtered[0][i]==acc0[i]);
    for(i=0;i<210;i++)assert(MPU9250_CalcAngles(acc0,gyro0,NULL,5,a));
    assert(MPU9250_CalcAngles(acc0,gyro0,mag,5,a));
    for(i=0;i<3;i++)assert(filtered[2][i]==mag[i]);
    /* Steady Nyquist-frequency noise gain alpha/(2-alpha), independently derived. */
    for(j=0;j<3;j++) {
        float hz=j==2?MPU9250_MAG_LPF_HZ:(j==1?MPU9250_GYRO_LPF_HZ:MPU9250_ACCEL_LPF_HZ);
        float dt=j==2?10:5;
        float alpha=dt/(dt+1000/(2*PI_F*hz));
        filter_ready[j]=0;LowPass3((uint8_t)j,gyro0,dt,hz);
        for(i=0;i<200;i++) {
            int16_t noise[3]={(int16_t)(i%2?1000:-1000),(int16_t)(i%2?-2000:2000),(int16_t)(i%2?3000:-3000)};
            LowPass3((uint8_t)j,noise,dt,hz);
        }
        assert(fabsf(filtered[j][0]-1000*alpha/(2-alpha))<.01f);
        assert(fabsf(filtered[j][1]+2000*alpha/(2-alpha))<.01f);
        assert(fabsf(filtered[j][2]-3000*alpha/(2-alpha))<.01f);
        LowPass3((uint8_t)j,gyro,dt,0);for(i=0;i<3;i++)assert(fabsf(filtered[j][i]-gyro[i])<.002f);
    }
    initialize();for(i=0;i<3;i++)assert(!filter_ready[i]);assert(mag_elapsed_ms==0);
    puts("PASS: all nine LPF channels, raw-unit precision, variable dt, 100Hz magnetic timing, shock rejection, bypass and reset");
}
static void test_fused_motion(void) {
    int axis,sign,i;int16_t acc[3],mag[3],gyro[3];float a[3],r,p,y;
    for(axis=0;axis<3;axis++)for(sign=-1;sign<=1;sign+=2) {
        initialize();memset(gyro,0,sizeof(gyro));sensors(0,0,0,acc,mag);
        assert(MPU9250_CalcAngles(acc,gyro,mag,0,a));gyro[axis]=(int16_t)(sign*738);
        for(i=1;i<=200;i++) {
            float theta=sign*45.0f*i*.005f;
            r=axis==0?theta:0;p=axis==1?theta:0;y=axis==2?theta:0;
            sensors(r,p,y,acc,mag);
            assert(MPU9250_CalcAngles(acc,gyro,(i%2)?NULL:mag,5,a));
            expect_pose(a,r,p,y,1.2f);
        }
        memset(gyro,0,sizeof(gyro));
        for(i=0;i<400;i++)assert(MPU9250_CalcAngles(acc,gyro,(i%2)?NULL:mag,5,a));
        expect_pose(a,axis==0?sign*45:0,axis==1?sign*45:0,axis==2?sign*45:0,.2f);
    }
    puts("PASS: fused positive/negative motion on all three axes with 200Hz IMU and 100Hz magnetic data");
}
static void test_late_mag(void) {
    int16_t acc[3],mag[3],gyro[3]={0,0,0};float a[3];int i;
    initialize();sensors(20,30,180,acc,mag);
    assert(MPU9250_CalcAngles(acc,gyro,NULL,0,a));expect_pose(a,20,30,0,.05f);
    for(i=0;i<2000;i++)assert(MPU9250_CalcAngles(acc,gyro,mag,5,a));
    printf("Late magnetic north 180deg away after 10s: pitch=%.3f roll=%.3f yaw=%.3f\n",a[0],a[1],a[2]);
    expect_pose(a,20,30,180,.3f);
    assert(fabsf(a[0]-30)<.05f && fabsf(a[1]-20)<.05f);
    /* MPU200Hz/AK8963 100Hz: pass NULL on alternate frames. */
    initialize();sensors(20,30,180,acc,mag);
    assert(MPU9250_CalcAngles(acc,gyro,NULL,0,a));
    for(i=0;i<4000;i++)assert(MPU9250_CalcAngles(acc,gyro,(i%2)?NULL:mag,5,a));
    printf("100Hz magnetic updates after 20s: yaw=%.3f\n",a[2]);
    expect_pose(a,20,30,180,.5f); /* 理想输入下大角差捕获后的暂态容差，不是实物精度承诺 */
    /* Purely vertical or zero field must not block gravity-only initialization. */
    initialize();acc[0]=acc[1]=0;acc[2]=16384;mag[0]=mag[1]=0;mag[2]=10000;
    assert(MPU9250_CalcAngles(acc,gyro,mag,0,a));expect_pose(a,0,0,0,.01f);
    initialize();memset(mag,0,sizeof(mag));assert(MPU9250_CalcAngles(acc,gyro,mag,0,a));
    puts("PASS: later magnetometer acquisition including 180deg, vertical field and zero field");
}
static void show_physical_limit(void) {
    int16_t acc[3],mag[3],gyro[3]={0,0,0};float a[3];int i;
    initialize();sensors(0,0,0,acc,mag);assert(MPU9250_CalcAngles(acc,gyro,mag,0,a));acc[0]=4915;
    for(i=0;i<2000;i++)assert(MPU9250_CalcAngles(acc,gyro,mag,5,a));
    printf("KNOWN LIMIT: level body + sustained 0.3g X acceleration -> estimated pitch %.3f deg\n",a[0]);
}
int main(void) {
    setvbuf(stdout,NULL,_IONBF,0);
    test_raw();test_lifecycle();test_static_poses();test_rotations();test_late_mag();test_nine_axis_lpf();test_fused_motion();show_physical_limit();
    puts("ALL TESTS PASSED");return 0;
}
