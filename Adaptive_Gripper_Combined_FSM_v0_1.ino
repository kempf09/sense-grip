/*
  ESP32-S3 Adaptive Gripper Combined Debugger + FSM v0
  Hardware: 6x FSR402, 4x VL53L0X, CF35-12 serial servo
  Libraries: Arduino-ESP32, Pololu VL53L0X

  Pin priority: ToF keeps GPIO 4..9. FSRs moved to ADC1 GPIO 1,2,3,10,11,12.
  IMPORTANT: confirm these pins are exposed and unused on the exact ESP32-S3 board.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>

// Paste the enum definition here:
enum State:uint8_t {IDLE,POSITIONING,GRASPING,HOLDING,RELEASING,PAUSED,FAULT};

// Optionally, define explicit prototypes to force the preprocessor to behave:
const char* stateName(State s);
void enterState(State s, const String& why);

// ---------------- Hardware mapping ----------------
static const uint8_t FSR_COUNT=6;
static const uint8_t FSR_PINS[FSR_COUNT]={1,2,4,5,6,10};
static const uint8_t TOF_COUNT=3;
static const int TOF_SDA=11, TOF_SCL=12;
static const int TOF_XSHUT[TOF_COUNT]={13,14,21};
static const uint8_t TOF_ADDR[TOF_COUNT]={0x30,0x31,0x32};
#define SERVO_RX 17
#define SERVO_TX 18

static const uint16_t ADC_SAMPLES=16;
static const float SUPPLY_MV=3300.0f, OPEN_OHMS=10000000.0f;
static const uint32_t FSR_PERIOD_MS=20, TOF_PERIOD_MS=50, MOTOR_PERIOD_MS=50;
static const uint16_t MAXD=128;

WebServer server(80);
HardwareSerial Bus(1);
VL53L0X tofDev[TOF_COUNT];
uint8_t servoId=1; uint32_t servoBaud=1000000;
bool motorWritesArmed=false; uint32_t motorArmUntil=0;
const uint32_t BAUDS[]={1000000,500000,250000,128000,115200,76800,57600,38400};
enum {PING=1,READ=2,WRITE=3,CAL=0x0B};

// ---------------- Persistent configuration ----------------
struct FsrConfig { float fixedOhms,f1,o1,f2,o2,zeroMv; };
struct FsrReading { uint16_t raw; float mv,ohms,force; };
FsrConfig fsrCfg[FSR_COUNT]; FsrReading fsr[FSR_COUNT];

struct TofConfig { uint32_t budgetUs,periodMs; bool enabled; char name[20]; };
struct TofReading { bool online=false,valid=false,stale=true; uint16_t mm=0; uint8_t status=0; uint32_t samples=0,errors=0,lastMs=0; String error; };
TofConfig tofCfg[TOF_COUNT]; TofReading tof[TOF_COUNT];

struct FsmConfig {
  uint32_t magic;
  bool closePositive,autoStart;
  int32_t openZero;
  uint16_t idleDetectMm,positionTargetMm,positionToleranceMm,maxFingerDeviationMm;
  float graspForceN,fsrSlipDropPct;
  uint16_t minimumActiveFsr,graspSpeed,positionSpeed,releaseSpeed,graspTorque,stallCurrentMa;
  uint8_t stallPercent,slipDeltaMm,slipConsecutive,maxRetries;
  uint32_t positioningTimeoutMs,graspTimeoutMs,releaseTimeoutMs,stallDebounceMs;
};
static const uint32_t FSM_MAGIC=0x46534D31;
FsmConfig fc;

void defaultFsm(){
  fc={FSM_MAGIC,true,false,0,150,5,3,8,2.0f,25.0f,3,30,20,30,200,2700,90,10,3,2,5000,7000,7000,150};
}
void loadAll(){
  Preferences p;
  p.begin("fsr402",true);
  for(uint8_t i=0;i<FSR_COUNT;i++){
    fsrCfg[i]={2000,1,30000,10,3000,0};
    String k="s"+String(i);
    fsrCfg[i].fixedOhms=p.getFloat((k+"r").c_str(),fsrCfg[i].fixedOhms);
    fsrCfg[i].f1=p.getFloat((k+"f1").c_str(),fsrCfg[i].f1); fsrCfg[i].o1=p.getFloat((k+"o1").c_str(),fsrCfg[i].o1);
    fsrCfg[i].f2=p.getFloat((k+"f2").c_str(),fsrCfg[i].f2); fsrCfg[i].o2=p.getFloat((k+"o2").c_str(),fsrCfg[i].o2);
    fsrCfg[i].zeroMv=p.getFloat((k+"z").c_str(),0);
  } p.end();
  p.begin("tof4",true);
  for(uint8_t i=0;i<TOF_COUNT;i++){
    tofCfg[i].budgetUs=p.getUInt(("b"+String(i)).c_str(),33000);
    tofCfg[i].periodMs=p.getUInt(("p"+String(i)).c_str(),50);
    tofCfg[i].enabled=p.getBool(("e"+String(i)).c_str(),true);
    String n=p.getString(("n"+String(i)).c_str(), i<3?"Finger "+String(i+1):"Palm");
    n.toCharArray(tofCfg[i].name,sizeof(tofCfg[i].name));
  } p.end();
  p.begin("fsm",true); size_t n=p.getBytesLength("cfg");
  if(n==sizeof(fc)) p.getBytes("cfg",&fc,sizeof(fc)); else defaultFsm();
  p.end(); if(fc.magic!=FSM_MAGIC) defaultFsm();
  p.begin("cf35",true); servoId=p.getUChar("id",1); servoBaud=p.getULong("baud",1000000); p.end();
}
void saveFsm(){ Preferences p;p.begin("fsm",false);p.putBytes("cfg",&fc,sizeof(fc));p.end(); }

// ---------------- CF35-12 protocol ----------------
struct Resp { bool ok=false; uint8_t id=0,st=0,n=0,d[MAXD]{}; String e; };
void busBegin(uint32_t b){servoBaud=b;Bus.end();delay(15);Bus.begin(b,SERIAL_8N1,SERVO_RX,SERVO_TX);delay(25);while(Bus.available())Bus.read();}
bool getb(uint8_t&v,uint32_t us=25000){uint32_t s=micros();while((uint32_t)(micros()-s)<us){if(Bus.available()){v=Bus.read();return true;}yield();}return false;}
Resp receivePacket(uint8_t want,uint8_t wantN=255){Resp r;uint8_t a=0,b=0;uint32_t s=micros();while((uint32_t)(micros()-s)<25000){if(Bus.available()){b=Bus.read();if(a==255&&b==255)break;a=b;}yield();}if(!(a==255&&b==255)){r.e="timeout";return r;}uint8_t len;if(!getb(r.id)||!getb(len)||!getb(r.st)){r.e="short header";return r;}if(len<2||len-2>MAXD){r.e="length";return r;}r.n=len-2;uint8_t sum=r.id+len+r.st;for(uint8_t i=0;i<r.n;i++){if(!getb(r.d[i])){r.e="short data";return r;}sum+=r.d[i];}uint8_t cs;if(!getb(cs)){r.e="checksum";return r;}r.ok=(uint8_t(~sum)==cs&&r.id==want&&(wantN==255||r.n==wantN));if(!r.ok)r.e="checksum/id/length";return r;}
Resp sendPacket(uint8_t id,uint8_t ins,const uint8_t*p,uint8_t n,bool reply=true,uint8_t rn=255){while(Bus.available())Bus.read();uint8_t l=n+2,s=id+l+ins;Bus.write(255);Bus.write(255);Bus.write(id);Bus.write(l);Bus.write(ins);for(uint8_t i=0;i<n;i++){Bus.write(p[i]);s+=p[i];}Bus.write(uint8_t(~s));Bus.flush();if(!reply){Resp r;r.ok=true;return r;}return receivePacket(id,rn);}
bool readBus(uint8_t id,uint8_t a,uint8_t n,uint8_t*out,String&e){uint8_t p[]={a,n};Resp r=sendPacket(id,READ,p,2,true,n);if(!r.ok){e=r.e;return false;}memcpy(out,r.d,n);return true;}
bool writeBus(uint8_t id,uint8_t a,const uint8_t*d,uint8_t n,String&e){uint8_t p[MAXD];p[0]=a;memcpy(p+1,d,n);Resp r=sendPacket(id,WRITE,p,n+1,true,0);if(!r.ok){e=r.e;return false;}return true;}
uint16_t u16(const uint8_t*p){return p[0]|uint16_t(p[1])<<8;} void p16(uint16_t v,uint8_t*p){p[0]=v;p[1]=v>>8;}
int16_t sm15(uint16_t v){return v&0x8000?-int16_t(v&0x7fff):int16_t(v);} int16_t sm10(uint16_t v){return v&0x400?-int16_t(v&0x3ff):int16_t(v);}
void psm(long v,uint8_t*p){uint16_t x=abs(v);if(v<0)x|=0x8000;p16(x,p);}
struct MotorReading { bool ok=false,moving=false,notReached=false; int16_t position=0,speed=0,current=0; float degrees=0,mA=0,load=0,voltage=0; uint8_t temp=0,status=0; String error; } motor;
bool pollMotor(){uint8_t b[18];String e;if(!readBus(servoId,56,18,b,e)){motor.ok=false;motor.error=e;return false;}motor.position=sm15(u16(b));motor.speed=sm15(u16(b+2));motor.load=sm10(u16(b+4))*.001f;motor.voltage=b[6]*.1f;motor.temp=b[7];motor.status=b[9];motor.moving=b[10]&1;motor.notReached=(b[10]>>1)&1;motor.current=sm15(u16(b+13));motor.mA=motor.current*6.5f;motor.degrees=motor.position*.087f;motor.ok=true;motor.error="";return true;}
bool setMode(uint8_t v){String e;return writeBus(servoId,33,&v,1,e);} bool torque(uint8_t v){String e;return writeBus(servoId,40,&v,1,e);}
bool motorPosition(long pos,long speed,long acc,long tq){uint8_t d[7];d[0]=acc;psm(pos,d+1);p16(tq,d+3);p16(speed,d+5);String e;return writeBus(servoId,41,d,7,e);}
bool motorSpeed(long speed,long acc,long tq){uint8_t d[7];d[0]=acc;p16(0,d+1);p16(tq,d+3);psm(speed,d+5);String e;return writeBus(servoId,41,d,7,e);}
void motorStop(){uint8_t z[2]={0,0};String e;writeBus(servoId,44,z,2,e);writeBus(servoId,40,z,1,e);}
int closeSign(){return fc.closePositive?1:-1;}
void closeAt(uint16_t speed,uint16_t tq){setMode(1);torque(1);motorSpeed(closeSign()*speed,20,tq);}
void openAt(uint16_t speed,uint16_t tq){setMode(1);torque(1);motorSpeed(-closeSign()*speed,20,tq);}

// ---------------- Sensors ----------------
float resistanceToForce(uint8_t i,float ohms){if(!isfinite(ohms)||ohms<=0||ohms>=OPEN_OHMS)return 0;float r1=fsrCfg[i].o1,r2=fsrCfg[i].o2,f1=fsrCfg[i].f1,f2=fsrCfg[i].f2;if(r1<=0||r2<=0||f1<=0||f2<=0||r1==r2)return NAN;float b=logf(f2/f1)/logf(r2/r1),a=f1/powf(r1,b);return a*powf(ohms,b);}
void sampleFsr(){for(uint8_t i=0;i<FSR_COUNT;i++){uint32_t sum=0;for(uint8_t n=0;n<ADC_SAMPLES;n++)sum+=analogRead(FSR_PINS[i]);fsr[i].raw=sum/ADC_SAMPLES;float measured=analogReadMilliVolts(FSR_PINS[i]);fsr[i].mv=fmaxf(0,measured-fsrCfg[i].zeroMv);if(fsr[i].mv<1)fsr[i].ohms=OPEN_OHMS;else if(fsr[i].mv>=SUPPLY_MV-1)fsr[i].ohms=0;else fsr[i].ohms=fsrCfg[i].fixedOhms*(SUPPLY_MV/fsr[i].mv-1);fsr[i].force=resistanceToForce(i,fsr[i].ohms);}}
float averageForce(uint8_t*activeOut=nullptr){float sum=0;uint8_t n=0;for(auto&r:fsr)if(isfinite(r.force)&&r.force>0.05f){sum+=r.force;n++;}if(activeOut)*activeOut=n;return n?sum/n:0;}
void shutdownTof(){for(uint8_t i=0;i<TOF_COUNT;i++){digitalWrite(TOF_XSHUT[i],LOW);pinMode(TOF_XSHUT[i],OUTPUT);tof[i]=TofReading();}delay(10);}
void initTof(){shutdownTof();Wire.end();Wire.begin(TOF_SDA,TOF_SCL,100000);Wire.setTimeOut(20);for(uint8_t i=0;i<TOF_COUNT;i++){if(!tofCfg[i].enabled){tof[i].error="disabled";continue;}pinMode(TOF_XSHUT[i],INPUT);delay(10);tofDev[i].setTimeout(500);if(!tofDev[i].init()){tof[i].error="init failed";digitalWrite(TOF_XSHUT[i],LOW);pinMode(TOF_XSHUT[i],OUTPUT);continue;}tofDev[i].setAddress(TOF_ADDR[i]);if(!tofDev[i].setMeasurementTimingBudget(tofCfg[i].budgetUs)){tof[i].error="budget rejected";continue;}tofDev[i].startContinuous(tofCfg[i].periodMs);tof[i].online=true;}}
void pollTof(){for(uint8_t i=0;i<TOF_COUNT;i++){if(!tof[i].online)continue;uint16_t mm=tofDev[i].readRangeContinuousMillimeters();if(tofDev[i].timeoutOccurred()){tof[i].errors++;tof[i].error="timeout";continue;}tof[i].mm=mm;tof[i].valid=true;tof[i].stale=false;tof[i].samples++;tof[i].lastMs=millis();tof[i].error="";}}

// ---------------- State machine ----------------

const char* stateName(State s){static const char*n[]={"IDLE","POSITIONING","GRASPING","HOLDING","RELEASING","PAUSED","FAULT"};return n[s];}
State state=IDLE,previousState=IDLE;uint32_t stateSince=0,stallSince=0;uint8_t retries=0,slipCount=0;bool fsmEnabled=false,manualOpen=false;String transitionReason="boot";uint16_t tofReference[TOF_COUNT]={};float holdForceReference=0;
void enterState(State s,const String&why){previousState=state;state=s;stateSince=millis();transitionReason=why;stallSince=0;if(s==IDLE){motorStop();torque(1);}else if(s==POSITIONING)closeAt(fc.positionSpeed,fc.graspTorque);else if(s==GRASPING)closeAt(fc.graspSpeed,fc.graspTorque);else if(s==HOLDING){motorStop();for(uint8_t i=0;i<TOF_COUNT;i++)tofReference[i]=tof[i].mm;holdForceReference=averageForce();slipCount=0;}else if(s==RELEASING)openAt(fc.releaseSpeed,fc.graspTorque);else if(s==PAUSED||s==FAULT)motorStop();}
bool fingerTofPositioned(){uint16_t mn=65535,mx=0;uint8_t n=0;for(uint8_t i=0;i<3;i++){if(!tof[i].valid)continue;mn=min(mn,tof[i].mm);mx=max(mx,tof[i].mm);n++;}if(n<2)return false;bool near=true;for(uint8_t i=0;i<3;i++)if(tof[i].valid&&abs((int)tof[i].mm-(int)fc.positionTargetMm)>fc.positionToleranceMm)near=false;return near&&(mx-mn)<=fc.maxFingerDeviationMm;}
bool slipDetected(){bool hit=false;for(uint8_t i=0;i<TOF_COUNT;i++)if(tof[i].valid&&abs((int)tof[i].mm-(int)tofReference[i])>fc.slipDeltaMm)hit=true;float f=averageForce();if(holdForceReference>0&&f<holdForceReference*(1.0f-fc.fsrSlipDropPct/100.0f))hit=true;if(hit)slipCount++;else slipCount=0;return slipCount>=fc.slipConsecutive;}
void updateFsm(){if(!fsmEnabled||manualOpen||state==PAUSED||state==FAULT)return;uint32_t elapsed=millis()-stateSince;uint8_t activeFsr=0;float force=averageForce(&activeFsr);bool highCurrent=motor.ok&&fabs(motor.mA)>=fc.stallCurrentMa*(fc.stallPercent/100.0f);if(highCurrent){if(!stallSince)stallSince=millis();}else stallSince=0;bool stalled=stallSince&&millis()-stallSince>=fc.stallDebounceMs;
  switch(state){
    case IDLE: if(fc.autoStart&&tof[3].valid&&tof[3].mm<fc.idleDetectMm)enterState(POSITIONING,"palm ToF threshold");break;
    case POSITIONING: if(fingerTofPositioned())enterState(GRASPING,"finger ToF aligned");else if(elapsed>fc.positioningTimeoutMs)enterState(GRASPING,"positioning timeout fallback");break;
    case GRASPING:
      if(force>=fc.graspForceN&&activeFsr>=fc.minimumActiveFsr)enterState(HOLDING,"force threshold reached");
      else if(stalled){motorStop();if(retries<fc.maxRetries){retries++;enterState(RELEASING,"stall without force; retry");}else enterState(FAULT,"stall retry limit");}
      else if(elapsed>fc.graspTimeoutMs)enterState(FAULT,"grasp timeout");break;
    case HOLDING: if(slipDetected())enterState(GRASPING,"ToF/FSR slip detected");break;
    case RELEASING:
      if(motor.ok&&abs(motor.position-fc.openZero)<=8){motorStop();if(retries)enterState(POSITIONING,"retry opened");else{fsmEnabled=false;enterState(IDLE,"release complete");}}
      else if(elapsed>fc.releaseTimeoutMs){motorStop();enterState(FAULT,"release timeout");}break;
    default:break;
  }
}

// ---------------- JSON/API helpers ----------------
String esc(String s){s.replace("\\","\\\\");s.replace("\"","\\\"");s.replace("\n","\\n");return "\""+s+"\"";}
void json(bool ok,const String&body,int code=200){server.sendHeader("Cache-Control","no-store");server.send(code,"application/json",String("{\"ok\":")+(ok?"true":"false")+","+body+"}");}
long argL(const char*n,long d=0){return server.hasArg(n)?strtol(server.arg(n).c_str(),nullptr,0):d;}
float argF(const char*n,float d=0){return server.hasArg(n)?server.arg(n).toFloat():d;}

void apiStatus(){String j="\"state\":"+esc(stateName(state))+",\"previous\":"+esc(stateName(previousState))+",\"reason\":"+esc(transitionReason)+",\"stateMs\":"+String(millis()-stateSince)+",\"fsmEnabled\":"+(fsmEnabled?String("true"):String("false"))+",\"manualOpen\":"+(manualOpen?String("true"):String("false"));j+=",\"motor\":{\"online\":"+(motor.ok?String("true"):String("false"))+",\"position\":"+String(motor.position)+",\"degrees\":"+String(motor.degrees,2)+",\"speed\":"+String(motor.speed)+",\"mA\":"+String(motor.mA,1)+",\"temp\":"+String(motor.temp)+",\"voltage\":"+String(motor.voltage,1)+"}";j+=",\"avgForce\":"+String(averageForce(),3)+",\"fsr\":[";for(uint8_t i=0;i<FSR_COUNT;i++){if(i)j+=',';j+="{\"raw\":"+String(fsr[i].raw)+",\"mv\":"+String(fsr[i].mv,1)+",\"ohms\":"+String(fsr[i].ohms,1)+",\"force\":"+(isfinite(fsr[i].force)?String(fsr[i].force,3):String("null"))+"}";}j+="] ,\"tof\":[";for(uint8_t i=0;i<TOF_COUNT;i++){if(i)j+=',';j+="{\"name\":"+esc(tofCfg[i].name)+",\"online\":"+(tof[i].online?String("true"):String("false"))+",\"valid\":"+(tof[i].valid?String("true"):String("false"))+",\"mm\":"+String(tof[i].mm)+",\"samples\":"+String(tof[i].samples)+",\"errors\":"+String(tof[i].errors)+",\"error\":"+esc(tof[i].error)+"}";}j+="]";json(true,j);}
void apiConfig(){String j="\"closePositive\":"+(fc.closePositive?String("true"):String("false"))+",\"autoStart\":"+(fc.autoStart?String("true"):String("false"))+",\"openZero\":"+String(fc.openZero)+",\"idleDetectMm\":"+String(fc.idleDetectMm)+",\"positionTargetMm\":"+String(fc.positionTargetMm)+",\"positionToleranceMm\":"+String(fc.positionToleranceMm)+",\"maxFingerDeviationMm\":"+String(fc.maxFingerDeviationMm)+",\"graspForceN\":"+String(fc.graspForceN,2)+",\"minimumActiveFsr\":"+String(fc.minimumActiveFsr)+",\"graspSpeed\":"+String(fc.graspSpeed)+",\"positionSpeed\":"+String(fc.positionSpeed)+",\"releaseSpeed\":"+String(fc.releaseSpeed)+",\"graspTorque\":"+String(fc.graspTorque)+",\"stallCurrentMa\":"+String(fc.stallCurrentMa)+",\"stallPercent\":"+String(fc.stallPercent)+",\"slipDeltaMm\":"+String(fc.slipDeltaMm)+",\"slipConsecutive\":"+String(fc.slipConsecutive)+",\"fsrSlipDropPct\":"+String(fc.fsrSlipDropPct,1)+",\"maxRetries\":"+String(fc.maxRetries)+",\"positioningTimeoutMs\":"+String(fc.positioningTimeoutMs)+",\"graspTimeoutMs\":"+String(fc.graspTimeoutMs)+",\"releaseTimeoutMs\":"+String(fc.releaseTimeoutMs)+",\"stallDebounceMs\":"+String(fc.stallDebounceMs);json(true,j);}
void saveConfigApi(){fc.closePositive=argL("closePositive",1);fc.autoStart=argL("autoStart",0);fc.idleDetectMm=constrain(argL("idleDetectMm",150),10,2000);fc.positionTargetMm=constrain(argL("positionTargetMm",5),1,500);fc.positionToleranceMm=constrain(argL("positionToleranceMm",3),0,100);fc.maxFingerDeviationMm=constrain(argL("maxFingerDeviationMm",8),0,500);fc.graspForceN=constrain(argF("graspForceN",2),0.05f,100.0f);fc.minimumActiveFsr=constrain(argL("minimumActiveFsr",3),1,6);fc.graspSpeed=constrain(argL("graspSpeed",30),1,32767);fc.positionSpeed=constrain(argL("positionSpeed",20),1,32767);fc.releaseSpeed=constrain(argL("releaseSpeed",30),1,32767);fc.graspTorque=constrain(argL("graspTorque",200),1,2047);fc.stallCurrentMa=constrain(argL("stallCurrentMa",2700),100,20000);fc.stallPercent=constrain(argL("stallPercent",90),10,100);fc.slipDeltaMm=constrain(argL("slipDeltaMm",10),1,255);fc.slipConsecutive=constrain(argL("slipConsecutive",3),1,20);fc.fsrSlipDropPct=constrain(argF("fsrSlipDropPct",25),1.0f,100.0f);fc.maxRetries=constrain(argL("maxRetries",2),0,10);fc.positioningTimeoutMs=constrain(argL("positioningTimeoutMs",5000),100,60000);fc.graspTimeoutMs=constrain(argL("graspTimeoutMs",7000),100,60000);fc.releaseTimeoutMs=constrain(argL("releaseTimeoutMs",7000),100,60000);fc.stallDebounceMs=constrain(argL("stallDebounceMs",150),20,5000);saveFsm();json(true,"\"saved\":true");}


// ---------------- Original FSR and CF35 configuration APIs ----------------
bool motorArmed(){ if(motorWritesArmed && (int32_t)(millis()-motorArmUntil)>=0) motorWritesArmed=false; return motorWritesArmed; }
void saveOneFsr(uint8_t i){
  Preferences p;p.begin("fsr402",false);String k="s"+String(i);
  p.putFloat((k+"r").c_str(),fsrCfg[i].fixedOhms);p.putFloat((k+"f1").c_str(),fsrCfg[i].f1);p.putFloat((k+"o1").c_str(),fsrCfg[i].o1);
  p.putFloat((k+"f2").c_str(),fsrCfg[i].f2);p.putFloat((k+"o2").c_str(),fsrCfg[i].o2);p.putFloat((k+"z").c_str(),fsrCfg[i].zeroMv);p.end();
}
void apiFsrConfig(){String j="\"sensors\":[";for(uint8_t i=0;i<FSR_COUNT;i++){if(i)j+=',';j+="{\"r\":"+String(fsrCfg[i].fixedOhms,1)+",\"z\":"+String(fsrCfg[i].zeroMv,1)+",\"f1\":"+String(fsrCfg[i].f1,4)+",\"o1\":"+String(fsrCfg[i].o1,1)+",\"f2\":"+String(fsrCfg[i].f2,4)+",\"o2\":"+String(fsrCfg[i].o2,1)+"}";}j+="]";json(true,j);}
void apiFsrSave(){int i=argL("id",-1);if(i<0||i>=FSR_COUNT){json(false,"\"error\":\"invalid sensor id\"",400);return;}float r=argF("r"),z=argF("z"),f1=argF("f1"),o1=argF("o1"),f2=argF("f2"),o2=argF("o2");if(r<=0||z<0||z>1000||f1<=0||f2<=0||o1<=0||o2<=0||o1==o2){json(false,"\"error\":\"invalid calibration values\"",400);return;}fsrCfg[i]={r,f1,o1,f2,o2,z};saveOneFsr(i);json(true,"\"saved\":"+String(i));}
void apiFsrZero(){int i=argL("id",-1);if(i<0||i>=FSR_COUNT){json(false,"\"error\":\"invalid sensor id\"",400);return;}fsrCfg[i].zeroMv+=fsr[i].mv;saveOneFsr(i);json(true,"\"zeroMv\":"+String(fsrCfg[i].zeroMv,1));}
void apiMotorInfo(){uint8_t b[40];String e;if(!readBus(servoId,0,40,b,e)){json(false,"\"error\":"+esc(e),400);return;}String x="\"id\":"+String(b[5])+",\"baudIndex\":"+String(b[6])+",\"secondaryId\":"+String(b[7])+",\"responseLevel\":"+String(b[8])+",\"minAngle\":"+String(u16(b+9))+",\"maxAngle\":"+String(u16(b+11))+",\"maxTemp\":"+String(b[13])+",\"maxVoltageRaw\":"+String(b[14])+",\"minVoltageRaw\":"+String(b[15])+",\"maxTorque\":"+String(u16(b+16))+",\"protectCurrent\":"+String(u16(b+28))+",\"mode\":"+String(b[33]);json(true,x);}
void apiMotorMemory(){uint8_t b[87];String e;if(!readBus(servoId,0,87,b,e)){json(false,"\"error\":"+esc(e),400);return;}String x="\"bytes\":[";for(uint8_t i=0;i<87;i++){if(i)x+=',';x+=String(b[i]);}x+="]";json(true,x);}
bool persistMotor(uint8_t id,uint8_t address,uint8_t value,uint8_t newId,uint32_t newBaud,String &e){if(!motorArmed()){e="writes not armed";return false;}uint8_t zero=0,one=1;if(!writeBus(id,40,&zero,1,e)||!writeBus(id,55,&zero,1,e)||!writeBus(id,address,&value,1,e))return false;if(newBaud&&newBaud!=servoBaud)busBegin(newBaud);uint8_t readback;if(!readBus(newId,address,1,&readback,e)||readback!=value){writeBus(newId,55,&one,1,e);return false;}return writeBus(newId,55,&one,1,e);}
void apiMotorArm(){if(server.arg("token")!="WRITE"){json(false,"\"error\":\"type WRITE to arm\"",400);return;}motorWritesArmed=true;motorArmUntil=millis()+60000;json(true,"\"armedSeconds\":60");}
void apiMotorSetId(){long v=argL("value",-1);if(v<0||v>253){json(false,"\"error\":\"invalid id\"",400);return;}String e;if(!persistMotor(servoId,5,v,v,0,e)){json(false,"\"error\":"+esc(e),400);return;}servoId=v;Preferences p;p.begin("cf35",false);p.putUChar("id",servoId);p.end();json(true,"\"id\":"+String(servoId));}
void apiMotorSetBaud(){long v=argL("value",-1);if(v<0||v>7){json(false,"\"error\":\"invalid baud index\"",400);return;}String e;if(!persistMotor(servoId,6,v,servoId,BAUDS[v],e)){json(false,"\"error\":"+esc(e),400);return;}Preferences p;p.begin("cf35",false);p.putULong("baud",servoBaud);p.end();json(true,"\"baud\":"+String(servoBaud));}

// ---------------- Integrated web UI ----------------
const char PAGE[] PROGMEM=R"HTML(<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1"><title>Adaptive Gripper Lab</title><style>
:root{--bg:#09101e;--p:#131d32;--l:#2a3958;--t:#edf4ff;--m:#9baccb;--b:#42d3ff;--g:#44df8d;--y:#ffc857;--r:#ff5d76}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--t);font:14px system-ui}.top{position:sticky;top:0;z-index:5;background:#0c1529f5;padding:10px 15px;border-bottom:1px solid var(--l);display:flex;gap:9px;align-items:center;flex-wrap:wrap}.brand{font-size:19px;font-weight:800}.pill{background:#1b2945;border:1px solid #304264;border-radius:8px;padding:6px 9px}.layout{display:grid;grid-template-columns:230px 1fr;min-height:100vh}nav{padding:12px;background:#0d1729;border-right:1px solid var(--l)}nav button{width:100%;padding:10px;text-align:left;background:transparent;color:var(--t);border:0;border-radius:8px;cursor:pointer}nav button.sel,nav button:hover{background:#1d2c4a;color:var(--b)}main{padding:18px;max-width:1450px}.page{display:none}.page.on{display:block}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(270px,1fr));gap:12px}.card{background:var(--p);border:1px solid var(--l);border-radius:12px;padding:15px}.metric{display:flex;justify-content:space-between;background:#0f1929;padding:7px;border-radius:7px;margin:5px 0}.value{font-size:28px;color:var(--b);font-weight:750}.act{border:0;border-radius:8px;padding:9px 13px;margin:3px;background:#2877ed;color:white;cursor:pointer}.green{background:#168c57}.red{background:#c83c55}.yellow{background:#a56c00}label{display:block;margin:8px 0}input,select{background:#0a1222;border:1px solid #3a4b6d;color:var(--t);border-radius:7px;padding:7px;width:100%;max-width:220px}.states{display:flex;flex-wrap:wrap;gap:8px;margin:12px 0}.state{padding:12px;border:1px solid var(--l);border-radius:9px;background:#0f1929}.state.active{background:#12613d;border-color:var(--g);box-shadow:0 0 14px #44df8d55}.log{white-space:pre-wrap;background:#070c16;padding:10px;border-radius:8px;min-height:90px;overflow:auto;max-height:350px}.bad{color:var(--r)}.good{color:var(--g)}table{border-collapse:collapse;width:100%}td,th{padding:6px;border-bottom:1px solid var(--l);text-align:left}.divider{border:0;border-top:1px solid var(--l);margin:12px 0}@media(max-width:760px){.layout{grid-template-columns:1fr}nav{display:flex;overflow:auto}nav button{min-width:140px}}
</style></head><body><header class=top><span class=brand>Adaptive Gripper Lab</span><span id=conn class="pill bad">OFFLINE</span><span class=pill>FSM <b id=stateTop>--</b></span><span class=pill>Pos <b id=posTop>--</b></span><button class="act red" onclick=emergency()>EMERGENCY STOP</button></header><div class=layout><nav id=nav></nav><main>
<section id=dash class="page on"><h1>Combined Dashboard</h1><div class=grid><div class=card><h3>Motor</h3><div class=metric><span>Position</span><b id=mpos>--</b></div><div class=metric><span>Current</span><b id=mcur>--</b></div><div class=metric><span>Temperature</span><b id=mtemp>--</b></div><div class=metric><span>Voltage</span><b id=mvolt>--</b></div></div><div class=card><h3>FSM</h3><div class=value id=stateMain>--</div><div id=reason>--</div><button class="act green" onclick=startCycle()>Start grasp cycle</button><button class=act onclick=releaseCycle()>Release</button></div></div><h2>ToF sensors</h2><div id=tofCards class=grid></div><h2>FSR sensors</h2><div id=fsrCards class=grid></div></section>
<section id=fsm class=page><h1>State Machine</h1><div class=states id=states></div><div class=grid><div class=card><h3>Controls</h3><button class="act green" onclick=startCycle()>Start</button><button class=act onclick=releaseCycle()>Release</button><button class="act yellow" onclick=pauseCycle()>Pause</button><button class=act onclick=resetCycle()>Reset</button><button class="act red" onclick=emergency()>Emergency stop</button><div class=metric><span>State time</span><b id=stateMs>--</b></div><div class=metric><span>Transition reason</span><b id=fsmReason>--</b></div></div><div class=card><h3>Open-position setup</h3><p>Open continuously, stop from live input, fine-adjust, then store the encoder position.</p><button class="act green" onclick=openStart()>Open continuously</button><button class="act red" onclick=openStop()>Stop opening</button><br><button class=act onclick="jog(-1)">Jog -</button><button class=act onclick="jog(1)">Jog +</button><button class="act green" onclick=setZero()>Set 0 position</button><div class=metric><span>Saved open position</span><b id=openZero>--</b></div></div></div><div class=card><h3>Editable parameters</h3><form id=cfgForm class=grid></form><button class="act green" onclick=saveCfg()>Save FSM configuration</button></div></section>
<section id=fsrCfgPage class=page><h1>FSR Calibration</h1><p>Original per-sensor resistor, zero-offset and two-point force calibration controls.</p><div id=fsrCfgGrid class=grid></div></section>
<section id=motorCfgPage class=page><h1>CF35-12 Configuration</h1><div class=grid><div class=card><h3>Dynamic motor control</h3><label>Operating mode<select id=modeSel onchange=switchMotorMode()><option value=0>Position mode</option><option value=1>Constant speed</option><option value=2>Constant current</option></select></label><label><input type=checkbox id=reverseDir style="width:auto"> Reverse direction</label><div id=mc0><label>Position<input id=mcPos type=number value=2048></label><label>Speed<input id=mcSpeed0 type=number value=30></label><label>Acceleration<input id=mcAcc0 type=number value=20></label><label>Torque<input id=mcTorque0 type=number value=200></label><button class="act green" onclick=runMotorCfg(0)>Move to position</button></div><div id=mc1 style="display:none"><label>Speed<input id=mcSpeed1 type=number value=30></label><label>Acceleration<input id=mcAcc1 type=number value=20></label><label>Torque<input id=mcTorque1 type=number value=200></label><button class="act green" onclick=runMotorCfg(1)>Run speed</button></div><div id=mc2 style="display:none"><label>Current command<input id=mcCurrent type=number value=100></label><button class="act green" onclick=runMotorCfg(2)>Apply current</button></div><button class="act red" onclick=post('/api/motor/stop')>Stop motor</button></div><div class=card><h3>Device settings</h3><button class="act red" onclick=armWrites()>Arm EEPROM writes</button><label>New servo ID<input id=newMotorId type=number min=0 max=253 value=2></label><button class=act onclick=setMotorId()>Change ID</button><label>Baud index<select id=newBaud><option value=0>0: 1M</option><option value=1>1: 500k</option><option value=2>2: 250k</option><option value=3>3: 128k</option><option value=4>4: 115200</option><option value=5>5: 76800</option><option value=6>6: 57600</option><option value=7>7: 38400</option></select></label><button class=act onclick=setMotorBaud()>Change baud</button></div></div><div class=grid style="margin-top:12px"><div class=card><h3>Configuration info</h3><button class=act onclick=loadMotorInfo()>Read configuration</button><div id=motorInfo class=log></div></div><div class=card><h3>Memory browser</h3><button class=act onclick=loadMotorMemory()>Read addresses 0-86</button><div class=log><table><thead><tr><th>Address</th><th>Hex</th><th>Raw</th></tr></thead><tbody id=motorMemory></tbody></table></div></div></div></section>
<section id=diag class=page><h1>Sensor diagnostics</h1><button class=act onclick="post('/api/tof/reinit')">Reinitialize ToF</button><button class=act onclick=scan()>Scan I2C</button><div id=scanOut class=log></div><div id=diagCards class=grid></div></section>
</main></div><script>
const $=id=>document.getElementById(id);const pages=[['dash','Dashboard'],['fsm','State Machine'],['fsrCfgPage','FSR Config'],['motorCfgPage','CF Motor Config'],['diag','Diagnostics']];nav.innerHTML=pages.map((p,i)=>`<button class="${i?'':'sel'}" onclick="show('${p[0]}',this)">${p[1]}</button>`).join('');function show(id,b){document.querySelectorAll('.page').forEach(x=>x.classList.remove('on'));$(id).classList.add('on');document.querySelectorAll('nav button').forEach(x=>x.classList.remove('sel'));b.classList.add('sel')}
async function req(u,o){let r=await fetch(u,{cache:'no-store',...o});let j=await r.json();if(!r.ok||!j.ok)throw Error(j.error||'request failed');return j}async function post(u,body=''){return req(u,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body})}
const stateNames=['IDLE','POSITIONING','GRASPING','HOLDING','RELEASING','PAUSED','FAULT'];states.innerHTML=stateNames.map(s=>`<div class=state id="st${s}">${s}</div>`).join('');
const fields=[['closePositive','Close direction positive','checkbox'],['autoStart','Automatic palm-ToF start','checkbox'],['idleDetectMm','Palm start threshold mm'],['positionTargetMm','Positioning target mm'],['positionToleranceMm','Position tolerance mm'],['maxFingerDeviationMm','Max finger deviation mm'],['graspForceN','Average grasp force N'],['minimumActiveFsr','Minimum active FSRs'],['positionSpeed','Positioning speed'],['graspSpeed','Grasp speed'],['releaseSpeed','Release speed'],['graspTorque','Torque limit'],['stallCurrentMa','100% stall current mA'],['stallPercent','Stall threshold %'],['stallDebounceMs','Stall debounce ms'],['slipDeltaMm','ToF slip delta mm'],['slipConsecutive','Consecutive slip samples'],['fsrSlipDropPct','FSR force-drop %'],['maxRetries','Maximum retries'],['positioningTimeoutMs','Position timeout ms'],['graspTimeoutMs','Grasp timeout ms'],['releaseTimeoutMs','Release timeout ms']];
async function loadCfg(){let c=await req('/api/config');cfgForm.innerHTML=fields.map(([k,l,t])=>`<label>${l}<input name="${k}" type="${t||'number'}" ${t==='checkbox'?(c[k]?'checked':''):`value="${c[k]}"`} step="any"></label>`).join('');openZero.textContent=c.openZero}async function saveCfg(){let p=new URLSearchParams(new FormData(cfgForm));fields.filter(x=>x[2]==='checkbox').forEach(x=>p.set(x[0],cfgForm.elements[x[0]].checked?'1':'0'));await post('/api/config',p);await loadCfg()}
async function initFsrCfg(){let c=await req('/api/fsr/config');fsrCfgGrid.innerHTML=c.sensors.map((s,i)=>`<div class=card><h3>FSR ${i+1}</h3><label>Fixed resistor ohm<input id="fr${i}" type=number value="${s.r}"></label><label>Zero offset mV<input id="fz${i}" type=number step=.1 value="${s.z}"></label><button class=act onclick="zeroFsr(${i})">Set current as zero</button><hr class=divider><label>Calibration point 1 N<input id="ff1${i}" type=number step=.01 value="${s.f1}"></label><label>Calibration point 1 ohm<input id="fo1${i}" type=number value="${s.o1}"></label><button class=act onclick="captureOhm(${i},1)">Use live resistance</button><label>Calibration point 2 N<input id="ff2${i}" type=number step=.01 value="${s.f2}"></label><label>Calibration point 2 ohm<input id="fo2${i}" type=number value="${s.o2}"></label><button class=act onclick="captureOhm(${i},2)">Use live resistance</button><br><button class="act green" onclick="saveFsr(${i})">Save sensor ${i+1}</button></div>`).join('')}function captureOhm(i,n){let card=window.lastStatus?.fsr?.[i];if(card)$(`fo${n}${i}`).value=card.ohms.toFixed(1)}async function zeroFsr(i){await post('/api/fsr/zero','id='+i);await initFsrCfg()}async function saveFsr(i){let p=new URLSearchParams({id:i,r:$('fr'+i).value,z:$('fz'+i).value,f1:$('ff1'+i).value,o1:$('fo1'+i).value,f2:$('ff2'+i).value,o2:$('fo2'+i).value});await post('/api/fsr/save',p);await initFsrCfg()}
async function startCycle(){await post('/api/fsm/start')}async function releaseCycle(){await post('/api/fsm/release')}async function pauseCycle(){await post('/api/fsm/pause')}async function resetCycle(){await post('/api/fsm/reset')}async function emergency(){await post('/api/fsm/estop')}async function openStart(){await post('/api/fsm/openStart')}async function openStop(){await post('/api/fsm/openStop')}async function jog(d){await post('/api/fsm/jog','d='+d)}async function setZero(){await post('/api/fsm/setzero');await loadCfg()}async function scan(){let j=await post('/api/tof/scan');scanOut.textContent=j.result}
function switchMotorMode(){[0,1,2].forEach(i=>$('mc'+i).style.display=(+modeSel.value===i?'block':'none'))}async function runMotorCfg(m){let d=reverseDir.checked?-1:1;if(m===0)await post('/api/motor/position',new URLSearchParams({position:+mcPos.value*d,speed:mcSpeed0.value,acc:mcAcc0.value,torque:mcTorque0.value}));if(m===1)await post('/api/motor/speed',new URLSearchParams({speed:+mcSpeed1.value*d,acc:mcAcc1.value,torque:mcTorque1.value}));if(m===2)await post('/api/motor/current',new URLSearchParams({current:+mcCurrent.value*d}))}async function armWrites(){let t=prompt('Type WRITE to arm EEPROM changes for 60 seconds');if(t==='WRITE')await post('/api/motor/arm','token=WRITE')}async function setMotorId(){await post('/api/motor/setid','value='+newMotorId.value)}async function setMotorBaud(){await post('/api/motor/setbaud','value='+newBaud.value)}async function loadMotorInfo(){motorInfo.textContent=JSON.stringify(await req('/api/motor/info'),null,2)}async function loadMotorMemory(){let j=await req('/api/motor/memory');motorMemory.innerHTML=j.bytes.map((v,i)=>`<tr><td>${i}</td><td>0x${i.toString(16).padStart(2,'0')}</td><td>${v}</td></tr>`).join('')}
function renderCards(s){window.lastStatus=s;tofCards.innerHTML=s.tof.map(x=>`<div class=card><h3>${x.name}</h3><div class=value>${x.valid?x.mm:'--'} mm</div><div class="${x.online?'good':'bad'}">${x.online?'ONLINE':'OFFLINE'}</div><small>Samples ${x.samples} | Errors ${x.errors} ${x.error||''}</small></div>`).join('');fsrCards.innerHTML=s.fsr.map((x,i)=>`<div class=card><h3>FSR ${i+1}</h3><div class=value>${x.force===null?'--':x.force.toFixed(2)} N</div><small>${x.raw} raw | ${x.ohms.toFixed(0)} ohm</small></div>`).join('');diagCards.innerHTML=tofCards.innerHTML+fsrCards.innerHTML}
async function poll(){try{let s=await req('/api/status');conn.textContent='ONLINE';conn.className='pill good';stateTop.textContent=stateMain.textContent=s.state;posTop.textContent=mpos.textContent=s.motor.position;mcur.textContent=s.motor.mA.toFixed(0)+' mA';mtemp.textContent=s.motor.temp+' C';mvolt.textContent=s.motor.voltage.toFixed(1)+' V';reason.textContent=fsmReason.textContent=s.reason;stateMs.textContent=s.stateMs+' ms';document.querySelectorAll('.state').forEach(x=>x.classList.remove('active'));$('st'+s.state)?.classList.add('active');renderCards(s)}catch(e){conn.textContent='OFFLINE';conn.className='pill bad'}setTimeout(poll,250)}loadCfg();initFsrCfg();poll();
</script></body></html>)HTML";

void setupRoutes(){
  server.on("/",HTTP_GET,[](){server.send_P(200,"text/html; charset=utf-8",PAGE);});
  server.on("/api/status",HTTP_GET,apiStatus);server.on("/api/config",HTTP_GET,apiConfig);server.on("/api/config",HTTP_POST,saveConfigApi);
  server.on("/api/fsr/config",HTTP_GET,apiFsrConfig);server.on("/api/fsr/save",HTTP_POST,apiFsrSave);server.on("/api/fsr/zero",HTTP_POST,apiFsrZero);
  server.on("/api/motor/info",HTTP_GET,apiMotorInfo);server.on("/api/motor/memory",HTTP_GET,apiMotorMemory);server.on("/api/motor/arm",HTTP_POST,apiMotorArm);server.on("/api/motor/setid",HTTP_POST,apiMotorSetId);server.on("/api/motor/setbaud",HTTP_POST,apiMotorSetBaud);
  server.on("/api/motor/current",HTTP_POST,[](){long t=argL("current");uint8_t d[2];psm(t,d);String e;if(!writeBus(servoId,44,d,2,e)){json(false,"\"error\":"+esc(e),400);return;}json(true,"\"command\":\"current\"");});
  server.on("/api/fsm/start",HTTP_POST,[](){fsmEnabled=true;manualOpen=false;retries=0;enterState(POSITIONING,"manual start");json(true,"\"started\":true");});
  server.on("/api/fsm/release",HTTP_POST,[](){fsmEnabled=true;manualOpen=false;retries=0;enterState(RELEASING,"release command");json(true,"\"releasing\":true");});
  server.on("/api/fsm/pause",HTTP_POST,[](){fsmEnabled=false;enterState(PAUSED,"manual pause");json(true,"\"paused\":true");});
  server.on("/api/fsm/reset",HTTP_POST,[](){fsmEnabled=false;manualOpen=false;retries=0;enterState(IDLE,"manual reset");json(true,"\"reset\":true");});
  server.on("/api/fsm/estop",HTTP_POST,[](){fsmEnabled=false;manualOpen=false;motorStop();enterState(FAULT,"emergency stop");json(true,"\"stopped\":true");});
  server.on("/api/fsm/openStart",HTTP_POST,[](){fsmEnabled=false;manualOpen=true;openAt(fc.releaseSpeed,fc.graspTorque);transitionReason="manual continuous opening";json(true,"\"opening\":true");});
  server.on("/api/fsm/openStop",HTTP_POST,[](){manualOpen=false;motorStop();transitionReason="manual opening stopped";json(true,"\"stopped\":true");});
  server.on("/api/fsm/jog",HTTP_POST,[](){manualOpen=false;motorStop();long d=constrain(argL("d",1),-1,1);pollMotor();setMode(0);torque(1);motorPosition(motor.position+d*10,10,10,fc.graspTorque);json(true,"\"jog\":"+String(d));});
  server.on("/api/fsm/setzero",HTTP_POST,[](){pollMotor();if(!motor.ok){json(false,"\"error\":\"motor offline\"",400);return;}fc.openZero=motor.position;saveFsm();json(true,"\"openZero\":"+String(fc.openZero));});
  server.on("/api/tof/reinit",HTTP_POST,[](){initTof();json(true,"\"reinitialized\":true");});
  server.on("/api/tof/scan",HTTP_POST,[](){String s="";for(uint8_t a=8;a<=0x77;a++){Wire.beginTransmission(a);if(Wire.endTransmission()==0){if(s.length())s+=' ';char b[6];snprintf(b,sizeof(b),"0x%02X",a);s+=b;}yield();}json(true,"\"result\":"+esc(s.length()?s:"none"));});
  // Direct debug motor endpoints retained for integration/testing.
  server.on("/api/motor/stop",HTTP_POST,[](){motorStop();json(true,"\"stopped\":true");});
  server.on("/api/motor/speed",HTTP_POST,[](){motorSpeed(argL("speed"),argL("acc",20),argL("torque",200));json(true,"\"command\":\"speed\"");});
  server.on("/api/motor/position",HTTP_POST,[](){setMode(0);torque(1);motorPosition(argL("position"),argL("speed",30),argL("acc",20),argL("torque",200));json(true,"\"command\":\"position\"");});
  server.onNotFound([](){server.send(404,"text/plain","Not found");});
}

void setup(){
  Serial.begin(115200);delay(300);loadAll();
  analogReadResolution(12);for(uint8_t i=0;i<FSR_COUNT;i++){pinMode(FSR_PINS[i],INPUT);analogSetPinAttenuation(FSR_PINS[i],ADC_11db);}
  busBegin(servoBaud);initTof();sampleFsr();pollMotor();stateSince=millis();
  WiFi.mode(WIFI_AP);WiFi.softAP("Gripper_Lab","12341234");Serial.print("Gripper dashboard: http://");Serial.println(WiFi.softAPIP());
  setupRoutes();server.begin();
}
void loop(){
  server.handleClient();uint32_t now=millis();static uint32_t lastFsr=0,lastTof=0,lastMotor=0;
  if(now-lastFsr>=FSR_PERIOD_MS){lastFsr=now;sampleFsr();}
  if(now-lastTof>=TOF_PERIOD_MS){lastTof=now;pollTof();}
  if(now-lastMotor>=MOTOR_PERIOD_MS){lastMotor=now;pollMotor();}
  updateFsm();delay(1);
}
