

#include "DSP28x_Project.h"
#include "math.h"
#include "stdio.h"
#include "stdint.h"
#include "Filter.h"
#define pi 3.14159265358979323846

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))


////////////////// MPPT /////////////////////


unsigned short data_num = 3;

unsigned short Trans_Ready = 0;
unsigned short Loop = 0;

float voltage = 0;
float current = 0;
float power = 0;
float current_mpp = 0;
int LED_On = 0;
int Rxnum = 0;
unsigned int Con_on = 0;
unsigned int Ready = 0;
unsigned int TxFifoLevel;
unsigned int RxfifoLevel;
unsigned int LoopCount;
unsigned short i;


int count = 0;
unsigned short on = 1;

unsigned short Error = 0;

int PI_con = 0;
int SW_F_Constant = 1875;
//int SW_F_Constant = 1875; //////////////////////////////////////////////////////////////////////
//int SW_F_Constant = 375;

float32 a = 1.1;

unsigned int Time_count = 0;
unsigned int LED_count = 0;
float Time = 0;
float LED_Time = 0;
int MPPT_Set = 0;

int LED_constant = 1;



////////////// Sensor Tune ///////////////////////////

float V_Max = 87.1586304; // tuned;
float V_offset = 0.0199999996; // tuned;

float HV_Max = 88.3; // tuned;
float HV_offset = 0; // tuned;


float A_Max = 69.7368011; // tuned;
float A_offset = -38.0250015; // tuned;

float V_sf = 0;

float HV_sf = 0;

float A_sf = 0;

////////////////////////////////////////////////////////////////////////////////////////

float PV1_V_present = 0, PV1_V_past = 0;
float PV1_I_present = 0, PV1_I_past = 0;
float PV1_W_present = 0, PV1_W_past = 0;
float Vaver1[128] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float Iaver1[128] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float PV1_V_aver_measured = 0;
float PV1_I_aver_measured = 0;
float PV1_W_aver_measured = 0;

float high_voltage = 0;
float high_voltage_max = 200;


////////////////////////////////// set //////////////////////////////////////


float V_ref = 30;
float V_del = 0.1;
float MPPT_Time = 0.1;
int MPPT_on = 0;


////////////////////////////////////////////////////////////////////////////////////////

float PV1_V_error = 0, PV1_V_error_Integ = 0, PV1_V_control_result = 0, PV1_V_control_sat = 0, AntiV1 = 0, diffV1 = 0, PV1_V_Pre_error = 0;

float PV1_Duty = 0;

float MAX_V = 48, MIN_V = 20;

//float constant = 0.00005;
float constant = 0.00005;

SOF_par LPF2_100_par = {0,0,0,0,0,0};
float LPF2_100_var[8] = {0,0,0,0,0,0,0,0};
val2 VI = {0,0};
val2 VI_res = {0,0};

//float Kp = 0.1, Ki = 100, Ka = 10, Kd = 0;
//float Kp = 0.01, Ki = 10, Ka = 100, Kd = 0; //- 20khz
//float Kp = 0.01, Ki = 100, Ka = 100, Kd = 0; // - 100khz
float Kp = 0.001, Ki = 10, Ka = 1000, Kd = 0; //- 20khz

void PV_V_PI(void);
void MPPT(void);


interrupt void ADC_PI(void);



void main(void)

 {
        // 전역 인터럽트 스위치 off
        DINT;

        /*
        1. watchdog off
        2. PLLCR 레지스터를 설정하여 SYSCLKOUT 을 설정
        3. 쿨럭을 ON 하여 주변 장치에 공급
        */
        InitSysCtrl();

        /*
        1. 개별 인터럽트 설정 플래그 비트를 모두 클리어 한다 -> 기존 인터럽트 플래그 비트를 초기화
        2. PIE 란 인터럽트를 배분하는 레지스터 즉, 여러 인터럽트들의 관리자 역할을 한다고 생각하면 됨
            --> https://pinkwink.kr/874
        3. PIE 벡터 테이블을 초기화 한다. PIE 활성화 작업으로, 모든 벡터 테이블을 TI가 정한 디폴트 인터럽트 서비스 루틴에 연결한다.
        4. IER (CPU 인터럽트 활성화 레지스터) 를 0으로 클리어 하여 사용불가 상태로 만듬
        5. IFR (CPU 인터럽트 플래그 레지스터) 를 0으로 클리어 하여 사용불가 상태로 만듬

        ex. IER = 0x0001; ====> 1번 인터럽트를 사용 가능으로 한다.
        */
        InitPieCtrl();
        InitPieVectTable();
        IER = 0x0000;
        IFR = 0x0000;


        // ADC 초기화
        InitAdc();

        /*
        디바이스의 모든 주변 장치를 초기화하며 해당 메서드는 DSP2833x_CpuTimers.c 에서 확인할 수 있다.
        DSP28335 에서 CPU Timer 1,2 는 BIOS 스케쥴링 등 이미 사용처가 명시되어 있다. 따라서 사용자는 Cpu Timer0 을 사용한다.

        CPU Timer 는 PLL 을 거친 SYSCLKOUT을 공급받아 동작한다.

        타이머가 동작 시 'PSCH.PSC' 는 TDDRH:TDDR의 값을 복사해온다.
        이후 SYSCLKOUT 의 한 쿨럭당 1씩 Down-counting 을 한다.
        이후 값이 0에서 -1이 되는 순간 'TIMH:TIM' 에 1쿨럭의 신호를 보내게 된다.

        'TIMH:TIM' 또한 'PRDH:PRD' 의 값을 복사해와 대기하다가 'PSCH' 로 부터 -1 신호를 받는 순간, Donw-counting 을 시작하고
        마찬가지로 0에서 -1이 되는 순간, TINT 신호를 보내 인터럽트를 발생시킨다.

        따라서 인터럽트의 주기는 (1/SYSCLKOUT) * (TDDRH:TDDR + 1) * (PRDH:PRD + 1) 이라고 볼 수 있으며 28335의 SYSCLKOUT 은
        150MHz 를 사용한다.
        */
        InitCpuTimers();

        InitScibGpio();


        /*
        CpuTimer 레지스터 초기화
        ConfigCpuTimer 메서드는 DSP2833x_CpuTimer.c 파일에서 확인할 수 있다.
        CpuTimer 의 주소, 주파수, 시간 (CPUTIMER_VARS *Timer, float Freq, float Period) 를 파라미터를 갖는다.

        메서드 내부를 확인해보면 Freq 의 단위는 MHz, Period 의 단위는 us 이다.
        즉, ConfigCpuTimer(&CpuTimer0, 150, 1000000) 에서
        Freq => 150MHz
        Period => 1초
        임을 알 수 있다.
        */
        ConfigCpuTimer(&CpuTimer0, 150, 1000000);        // CpuTimer0 - 150MHz

        // 해당 내용 이해 안됨
        CpuTimer0Regs.PRD.all = (15000000-1);           // CpuTimer0 PRD 포화방지

        /* ALLOW ~ EDIS 까지 한세트
        메모리 상의 값의 쓰기를 사용할 때 사용하는 것으로 생각됨
        MMR(Memory Mapped Register) 영역은 일반적으로 Protect가 걸려있으므로 이를 제거했다가 다시 걸어주는 것
        -> https://m.blog.naver.com/random61/60184200475
        */
        EALLOW;

        /*
        고속 CLOCK 설정
        HSPCLK = SYSCLKOUT / (HISPCP * 2)
                = 150MHz / (1 * 2) = 75MHz

        HSPCLK : High Speed Clock, LSPCLK : Low Speed Clock 으로
        HSPCLK 는 ADC, Event Manager 에 사용되고 LSPCLK 는 eCAN, McBSSP, SPI, SCI 에 사용된다.
        -> https://yeahhappyday.tistory.com/entry/DSP281xSysCtrlc-%EC%A2%80%EB%8D%94-%EC%9E%90%EC%84%B8%ED%9E%88
        */
        SysCtrlRegs.HISPCP.bit.HSPCLK = 1;

        /*
        ACD Interrupt 가 일어나면 ADC_PI 메서드를 실행시키겠다는 의미
        DSP2833x.PieVect.c 파일을 확인하면 PieVectTable 의 벡터에 대해서 찾아볼 수 있다.
        */
        PieVectTable.ADCINT = &ADC_PI;


        EDIS;





        EALLOW;


        /*
        기본적으로 특수 용도 핀을 제외한 모든 핀은 리셋 후 INPUT 상태로 설정된다.
        MUX : 핀의 기능 설정
        DIR : 핀의 방향 설정
        SEL : 핀 입력에 대한 자격 설정
        PUD : 핀의 내부 풀업 설정

        GPIO 핀에는 최대 3개까지의 서로 다른 기능이 탑재되어 있으며, 이는 MUX 를 통해 설정할 수 있다.
        00 : I/O,
        01 : EPWM,
        10 : MCLKRA,
        11 : ECAP
        기본적으로 MUX 와 PIN 을 지정하여 설정할 수 있지만, MUX1.all 등을 통해 해당 MUX 의 모든 핀을 설정할 수 있다.


        PUD 에서는 내부 Pull-up 저항의 Enable/Disable 을 설정한다.
        GPxPUD=1 이면 Pull-Up disable
        ePWM(GPIO 0~11) -> Default : Pull-Up Disable
        나머지 GPIO Pins -> Default : Pull-up Enable

        ----> https://blog.naver.com/realseobi/220470485269



        DAT : 이 레지스트의 비트가 0이면 LOW, 1이면 HIGH 로 3.3V가 감지된다
        ex.
        GpioCtrlRegs.GPADIR.bit.GPIO10 = 1
        GpioDataRegs.GPADAT.bit.GPIO8 = 0
        ======> 10번 핀 HIGH, 8번 핀 LOW 설정

        SET : 핀이 출력 상태로 정의 되어있을 때, 출력을 HIGH 로 만들어준다.
        ex.
        GpioCtrlRegs.GPADIR.bit.GPI8 = 1
        GpioCtrlRegs.GPADIR.bit.GPI9 = 1
        GpioCtrlRegs.GPASET.bit.GPIO8 = 1
        GpioDataRegs.GPASET.bit.GPI9 = 1
        =======> 8번 핀과 9번 핀을 출력 상태로 설정 (DIR) 후 SET 레지스터를 이용하여 촐력을 HIGH 로 설정

        CLEAR : SET 과 반대로 LOW 로 설정

        TOGGLE : 핀이 HIGH 일 경우 LOW 로, LOW 일 경우 HIGH 로 설정

        ------> https://velog.io/@emdydqortkgh/00

        333번 Line 까지의 설명
        */
        GpioCtrlRegs.GPAMUX1.all = 0;

        GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;


        GpioCtrlRegs.GPADIR.all = 0;


        GpioCtrlRegs.GPADIR.bit.GPIO7 = 0;  // 7번 핀 입력 설정
        GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;  // 8번 핀 출력 설정
        GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;  // 9번 핀 출력 설정

        GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;

        GpioCtrlRegs.GPADIR.bit.GPIO12 = 0;
        GpioCtrlRegs.GPADIR.bit.GPIO13 = 0;

        GpioDataRegs.GPADAT.bit.GPIO8 = 0;
        GpioDataRegs.GPADAT.bit.GPIO9 = 0;
        GpioDataRegs.GPADAT.bit.GPIO10 = 1;
        GpioDataRegs.GPADAT.bit.GPIO11 = 1;


        GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO18 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;


        GpioDataRegs.GPADAT.bit.GPIO16 = 0;
        GpioDataRegs.GPADAT.bit.GPIO17 = 0;
        GpioDataRegs.GPADAT.bit.GPIO18 = 0;
        GpioDataRegs.GPADAT.bit.GPIO19 = 0;
        GpioDataRegs.GPADAT.bit.GPIO20 = 0;
        GpioDataRegs.GPADAT.bit.GPIO21 = 0;



        GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
        GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;

        GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
        GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;

        GpioDataRegs.GPADAT.bit.GPIO31 = 0;
        GpioDataRegs.GPBDAT.bit.GPIO34 = 0;


        GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 12;
        GpioIntRegs.GPIOXINT2SEL.bit.GPIOSEL = 13;
//======================================================================


        /*
         ADCCLKPS : ADC 쿨럭 설정
         ADCCLK 는 ADCTRL3 레지스터의 ADCCLKPS 비트와 ADCCTL 레지스터의 CPS 비트에 따라 시스템 쿨럭에서 분주된 HSPCLK를 분주하여
         ADCCLK 를 만들고 ADCTRL1 레지스터의 ACQ_PS 비트에 따라 S/H 쿨럭 펄스 폭을 결정한다.
         SMODE_SEL : Sampling Mode 선택
         CONT_RUN : Continuous run
         ACQ_PS : S/H 사이클
         SEQ_CASC : 시퀀스 모드 설정 0 : 병렬, 1 : 직렬
         ADCCHSELSEQ1.bit.CONVN : ADC 순서 설정
        */

         AdcRegs.ADCTRL3.bit.ADCCLKPS = 1;
         AdcRegs.ADCTRL3.bit.SMODE_SEL = 1;
         AdcRegs.ADCTRL1.bit.CONT_RUN = 0;
         AdcRegs.ADCTRL1.bit.CPS = 2;

         AdcRegs.ADCTRL1.bit.ACQ_PS = 4;
         AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;
         AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 2;
         AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0;
         AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 1;
         AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 2;

        /*
         EPWM_SOCB_SEQ : EPWM compare B SOC enable
         SEQ1 Interrupt enable

        */
         AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ = 1;
         AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;

        /*
         TBCLK는 Time Base clock을 의미하며 pwm의 clock으로 사용하며 수식은
         TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV)

         EPwm1Regs.ETPS.bit.INTPRD : EPWMxINTn Period Select
         EPwm1Regs.TBCTL.bit.CTRMODE : Counter Mode
         EPwm1Regs.TBCTL.bit.HSPCLKDIV : High speed time pre-scale
         EPwm1Regs.TBCTL.bit.CLKDIV : Timebase clock pre-scale
        */

         EPwm1Regs.ETPS.bit.INTPRD = 1;
         EPwm1Regs.TBCTL.bit.CTRMODE = 2;
         EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1;
         EPwm1Regs.TBCTL.bit.CLKDIV = 0;


        /*
         ------> PWM 주기 설정 참고 https://yeahhappyday.tistory.com/entry/ePWM-%EC%84%A4%EC%A0%95

         EPwm1Regs.TBPRD : Period register set
         EPwm1Regs.TBCTR : Counter

         TBCTL 은 타임 베이스 서브모듈 컨트롤러, TBPRD 는 주기 설정 레지스터, 이 두개가 가장 중요하다.
         EPwm1Regs.ETSEL.bit.SOCAEN    = 0;    // Disable SOC on A group
         EPwm1Regs.ETSEL.bit.SOCASEL    = 4;   // Select SOC on up-count
         EPwm1Regs.ETPS.bit.SOCAPRD = 1;       // Generate pulse on 1st event
        */

         EPwm1Regs.TBPRD = SW_F_Constant;  // 설정값
         EPwm1Regs.TBCTR = 0x0000;
         EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
         EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
         EPwm1Regs.DBCTL.bit.POLSEL = 0;
         EPwm1Regs.CMPA.half.CMPA = 0;

         // --------------------------------------------------------------

         /* ADC 설정 예시
          AdcRegs.ADCTRL3.bit.ADCCLKPS = 3;      // ADCCLK = HSPCLK/(ADCCLKPS*2)/(CPS+1)
          AdcRegs.ADCTRL1.bit.CPS = 1;    // ADCCLK = 75MHz/(3*2)/(1+1) = 6.25MHz
          AdcRegs.ADCTRL1.bit.ACQ_PS = 3;    // 샘플/홀드 사이클 = ACQ_PS + 1 = 4 (ADCCLK기준)

          AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;   // 시퀀스 모드 설정: 직렬 시퀀스 모드 (0:병렬 모드, 1:직렬 모드)
          AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 0;  // ADC 채널수 설정: 1개(=MAX_CONV+1)채널을 ADC
          AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 4;   // ADC 순서 설정: 첫번째로 ADCINA2 채널을 ADC

          AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ = 1;   // ePWM_SOCB로 ADC 시퀀스 시동
          AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  // ADC 시퀀스 완료시 인터럽트 발생 설정

          //ePWM_SOCB 이벤트 트리거 설정
          EPwm3Regs.ETSEL.bit.SOCBEN = 1;    // SOCB 이벤트 트리거 Enable
          EPwm3Regs.ETSEL.bit.SOCBSEL = 2;   // SCCB 트리거 조건 : 카운터 주기 일치 시
          EPwm3Regs.ETPS.bit.SOCBPRD = 1;    // SOCB 이벤트 분주 설정 : 트리거 조건 한번 마다
          EPwm3Regs.TBCTL.bit.CTRMODE = 0;   // 카운트 모드 설정:
                                             // 0 : Up-count , 1 : Donw-count, 2 : Up-Down-count, 3 : Freeze

          EPwm3Regs.TBCTL.bit.HSPCLKDIV = 1;   // TBCLK = [SYSCLKOUT / ((HSPCLKDIV*2) * 2^(CLKDIV))]
          EPwm3Regs.TBCTL.bit.CLKDIV = 1;    // TBCLK = [150MHz / (2*2)] = 37.5MHz
             EPwm3Regs.TBPRD = 37499;     // TB주기= (TBPRD+1)/TBCLK = 37500/37.5MHz = 1us(1KHz)
          EPwm3Regs.TBCTR = 0x0000;     // TB 카운터 초기화

          // PIE의 ADC 인터럽트 활성화
          PieCtrlRegs.PIEIER1.bit.INTx6 = 1;   // PIE 인터럽트(ADCINT) 활성화
          IER |= M_INT1;         // CPU 인터럽트(INT1)  활성화


         -------> 출처 : https://blog.naver.com/cyjrntwkd/70124266599

        */




         // SOC 신호 혹은 인터럽트는 ADC 변환의 시작 신호를 의미한다.
         EPwm2Regs.ETSEL.bit.SOCBEN = 1;
         EPwm2Regs.ETSEL.bit.SOCBSEL = 2;
         EPwm2Regs.ETPS.bit.SOCBPRD = 1;


         EPwm2Regs.ETPS.bit.INTPRD = 1;
         EPwm2Regs.TBCTL.bit.CTRMODE = 2;
         EPwm2Regs.TBCTL.bit.HSPCLKDIV = 1;
         EPwm2Regs.TBCTL.bit.CLKDIV = 0;

         EPwm2Regs.TBPRD = 1875;// 20kHz 설정

         EPwm2Regs.TBCTR = 0x0000;              // TB 카운터 초기화

         /*
          AQ (Action Qualifier 는 스위칭 상태를 결정하는 레지스터
          CAU,CAD,CBU,CBD,ZRO,PRD 레지스터가 있다.
          - DSP 교재 PDF 기준 108 페이지 참고

          현재 CAU = AQ_CLEAR ( == 1 ) 는 UP COUNT MODE 일 때,COUNTER 값과 CMPA 레지스터의 값이 같으면 LOW 상태 출력,
          CAD = AQ_SET ( == 2 ) 는 DOWN COUNT MODE 일 때, COUNTER 값과 CMPA 레지스터의 값이 같으면 HIGH 상태 출력

          기본적으로 UP-DWON COUNT MODE 를 사용하는 경우, UP 일때와 DOWN 일 때 두 지점에서 CMP 값과 만나는 지점이 발생한다.
          그렇기 떄문에 UP 방향일 때인 CAU, DOWN 방향일 떄의 CAD 를 설정하여 스위칭 상태를 지정할 수 있다.

          마찬가지로 UP 혹은 DWON COUNT MODE 를 사용하는 경우에는 COUNTER = PRD 혹은 ZRO 가 스위칭 상태의 기준이 되기 때문에
          CAU + ZRO / ZRO + CAD 레지스터를 활용하여 스위칭 상태를 결정한다.
         */
         EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
         EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
         EPwm2Regs.DBCTL.bit.POLSEL = 0;
         EPwm2Regs.CMPA.half.CMPA = 0;


         EPwm1Regs.TBCTL.bit.PHSEN = 0;
         EPwm2Regs.TBCTL.bit.PHSEN = 1;

         EPwm1Regs.TBCTL.bit.SYNCOSEL = 2;
         EPwm2Regs.TBCTL.bit.SYNCOSEL = 2;


        EDIS;


        XIntruptRegs.XINT1CR.bit.POLARITY = 2;
        XIntruptRegs.XINT1CR.bit.ENABLE = 1;

        XIntruptRegs.XINT2CR.bit.POLARITY = 2;
        XIntruptRegs.XINT2CR.bit.ENABLE = 1;

        PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
        PieCtrlRegs.PIEIER1.bit.INTx5 = 1;
        PieCtrlRegs.PIEIER1.bit.INTx6 = 1;     // ADCINT Interrupt On



        IER |= M_INT1;


        EINT;
        ERTM;
        EnableInterrupts();

        LPF2_100_par = LPF2_set(100.0, 0.7071, 0.00002);

        V_sf = V_Max / 65535.0;
        HV_sf = HV_Max / 65535.0;
        A_sf = A_Max / 65535.0;



        while(1)
            {


            }
    }


interrupt void ADC_PI(void)
{

    GpioDataRegs.GPADAT.bit.GPIO15 = 1;

    Time_count++;
    Time = constant * Time_count;
    if(Time >= MPPT_Time) {MPPT_Set = 1; Time = 0; Time_count = 0;}

    voltage = (AdcRegs.ADCRESULT0 - AdcRegs.ADCRESULT1) * V_sf + V_offset;
    current = AdcRegs.ADCRESULT4 * A_sf + A_offset;
    VI.a = voltage;
    VI.b = current;


    if(current <= 0) current = 0;

    VI_res = SOF2(VI, LPF2_100_var, LPF2_100_par);


    PV1_V_aver_measured -= Vaver1[count];
    Vaver1[count] = VI_res.a * 0.0078125;
    PV1_V_aver_measured += Vaver1[count];

    PV1_I_aver_measured -= Iaver1[count];
    Iaver1[count] = VI_res.b * 0.0078125;
    PV1_I_aver_measured += Iaver1[count];

    count++;
    count = count & 0x7f;

    if(Con_on == 0)
    {
        V_ref = voltage;
        EPwm1Regs.CMPA.half.CMPA = 0;
        V_ref = voltage;
        PV1_V_error = 0;
        PV1_V_error_Integ = 0;
        PV1_V_control_result = 0;
        PV1_V_control_sat = 0;
        PV1_Duty = 0;
        MPPT_on = 0;
        MPPT_Set = 0;
        LED_constant = 1;
    }


    if(Con_on == 1)
    {

        if( MPPT_Set == 1 && MPPT_on)  MPPT();

        PV_V_PI();


    }


    GpioDataRegs.GPADAT.bit.GPIO15 = 0;

    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void PV_V_PI(void)
{

    PV1_V_error = voltage - V_ref;

    PV1_V_error_Integ = PV1_V_error_Integ + (Ki * PV1_V_error *constant);

    if(PV1_V_error_Integ <= 0) PV1_V_error_Integ = 0;
    if(PV1_V_error_Integ >= 0.9) PV1_V_error_Integ = 0.9;

    PV1_V_control_result = Kp * PV1_V_error + PV1_V_error_Integ;

    if(PV1_V_control_result >= 0.9) PV1_V_control_sat = 0.9;
    else if(PV1_V_control_result <= 0) PV1_V_control_sat = 0;
    else PV1_V_control_sat = PV1_V_control_result;

    PV1_Duty = (Uint16) (SW_F_Constant*PV1_V_control_sat);        //Duty


    EPwm1Regs.CMPA.half.CMPA = PV1_Duty;

}

void MPPT(void)
{

    PV1_W_aver_measured = PV1_V_aver_measured * PV1_I_aver_measured;





    if(PV1_W_past == PV1_W_aver_measured) V_ref = V_ref;

    if (PV1_W_past < PV1_W_aver_measured)
        {
            if(PV1_V_past < PV1_V_aver_measured) V_ref = V_ref + V_del;
            if(PV1_V_past > PV1_V_aver_measured) V_ref = V_ref - V_del;
        }

    if (PV1_W_past > PV1_W_aver_measured)
        {
            if(PV1_V_past < PV1_V_aver_measured) V_ref = V_ref - V_del;
            if(PV1_V_past > PV1_V_aver_measured) V_ref = V_ref + V_del;
        }


    if(V_ref >= MAX_V) V_ref = MAX_V;
    if(V_ref <= MIN_V) V_ref = MIN_V;


        PV1_V_past = PV1_V_aver_measured;
        PV1_W_past = PV1_W_aver_measured;

        MPPT_Set = 0;

}




