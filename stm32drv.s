
;��������---------
BIT0                    EQU             0X00000001
BIT1                    EQU             0X00000002
BIT2                    EQU             0X00000004
BIT3                    EQU             0X00000008
BIT4                    EQU             0X00000010
BIT5                    EQU             0X00000020
BIT6                    EQU             0X00000040
BIT7                    EQU             0X00000080
BIT8                    EQU             0X00000100
BIT9                    EQU             0X00000200
BIT10                   EQU             0X00000400
BIT11                   EQU             0X00000800
BIT12                   EQU             0X00001000
BIT13                   EQU             0X00002000
BIT14                   EQU             0X00004000
BIT15                   EQU             0X00008000
BIT16                   EQU             0X00010000
BIT17                   EQU             0X00020000
BIT18                   EQU             0X00040000
BIT19                   EQU             0X00080000
BIT20                   EQU             0X00100000
BIT21                   EQU             0X00200000
BIT22                   EQU             0X00400000
BIT23                   EQU             0X00800000
BIT24                   EQU             0X01000000
BIT25                   EQU             0X02000000
BIT26                   EQU             0X04000000
BIT27                   EQU             0X08000000
BIT28                   EQU             0X10000000
BIT29                   EQU             0X20000000
BIT30                   EQU             0X40000000
BIT31                   EQU             0X80000000

GPIOB      EQU 0X40011C00  ;GPIOB ��ַ
GPIOB_CRL  EQU 0X40011C00  ;�����üĴ���
GPIOB_CRH  EQU 0X40011C04  ;�����üĴ���
GPIOB_ODR  EQU 0X40011C0C  ;�����ƫ�Ƶ�ַ0Ch
GPIOB_BSRR EQU 0X40011C10  ;����λ�������ƫ�Ƶ�ַ10h
GPIOB_BRR  EQU 0X40011C14  ;�����ƫ�Ƶ�ַ14h
IOPBEN        EQU BIT7           ;GPIOBʹ��λ

GPIOD      EQU 0X40011C00  ;GPIOD ��ַ
GPIOD_CRL  EQU 0X40011C00  ;�����üĴ���
GPIOD_CRH  EQU 0X40011C04  ;�����üĴ���
GPIOD_ODR  EQU 0X40011C0C  ;�����ƫ�Ƶ�ַ0Ch
GPIOD_BSRR EQU 0X40011C10  ;����λ�������ƫ�Ƶ�ַ10h
GPIOD_BRR  EQU 0X40011C14  ;�����ƫ�Ƶ�ַ14h
IOPDEN        EQU BIT7           ;GPIODʹ��λ



;GPIO�Ĵ�����ַӳ��
GPIOC_BASE              EQU             0x40011000
GPIOC_CRL               EQU             (GPIOC_BASE + 0x00)
GPIOC_CRH               EQU             (GPIOC_BASE + 0x04)
GPIOC_IDR               EQU             (GPIOC_BASE + 0x08)
GPIOC_ODR               EQU             (GPIOC_BASE + 0x0C)
GPIOC_BSRR              EQU             (GPIOC_BASE + 0x10)
GPIOC_BRR               EQU             (GPIOC_BASE + 0x14)
GPIOC_LCKR              EQU             (GPIOC_BASE + 0x18)
;RCC�Ĵ�����ַӳ��
RCC_BASE                EQU             0x40021000
RCC_CR                  EQU             (RCC_BASE + 0x00)
RCC_CFGR                EQU             (RCC_BASE + 0x04)
RCC_CIR                 EQU             (RCC_BASE + 0x08)
RCC_APB2RSTR            EQU             (RCC_BASE + 0x0C)
RCC_APB1RSTR            EQU             (RCC_BASE + 0x10)
RCC_AHBENR              EQU             (RCC_BASE + 0x14)
RCC_APB2ENR             EQU             (RCC_BASE + 0x18)
RCC_APB1ENR             EQU             (RCC_BASE + 0x1C)
RCC_BDCR                EQU             (RCC_BASE + 0x20)
RCC_CSR                 EQU             (RCC_BASE + 0x24)

;STACK_TOP EQU 0X20002000

;��ջ��ʼ��
Stack_Size      EQU     0x00000400
                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp

Heap_Size       EQU     0x00000200
                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

;��ջ8�ֽڶ���
                PRESERVE8
;ʹ��THUMBָ��
                THUMB
;��λ����
    AREA RESET,CODE,READONLY
    ;DCD STACK_TOP ;MSP����ջָ��
    ;DCD START      ;��λ��PC��ʼֵ
__Vectors       DCD     __initial_sp               ; Top of Stack
                DCD     Reset_Handler              ; Reset Handler
                DCD     NMI_Handler                ; NMI Handler
                DCD     HardFault_Handler          ; Hard Fault Handler
                DCD     MemManage_Handler          ; MPU Fault Handler
                DCD     BusFault_Handler           ; Bus Fault Handler
                DCD     UsageFault_Handler         ; Usage Fault Handler
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     SVC_Handler                ; SVCall Handler
                DCD     DebugMon_Handler           ; Debug Monitor Handler
                DCD     0                          ; Reserved
                DCD     PendSV_Handler             ; PendSV Handler
                DCD     SysTick_Handler            ; SysTick Handler
__Vectors_End


        AREA    |.text|, CODE, READONLY
        ENTRY         ;ָʾ��ʼִ��
Reset_Handler		
	; Clear output
	EOR R0,R0	   ;
	LDR R1,=GPIOB
	STR R0,[R1]		   
	
	;Init begin

    ;BL.W   RCC_CONFIG_48MHZ

    LDR    R1,=RCC_APB2ENR
    LDR    R0,[R1]        ;��
    LDR    R2,=IOPBEN
    ORR    R0,R2        ;��
    STR    R0,[R1]        ;д��ʹ��GPIOBʱ��
    ;  ���������50MHz
    MOV    R0,#0x33
    LDR    R1,=GPIOB_CRH ;PB.8\9 �ڸ߼Ĵ���
    STR    R0,[R1]
    MOV    R0,#0x33333333
    LDR    R1,=GPIOB_CRL ;PB.0-7�ͼĴ���
    STR    R0,[R1]

    LDR    R1,=GPIOB_ODR
    LDR    R2,=0x00000000
    STR    R2,[R1]
    ;MOV    R3,#1
    ;B      GOON


	MOV R6,#0
	MOV R7, #2400 ; 10KHz 2400
	MOV R8,#0
	MOV R9,#10
	LDR R10,=0x123456 ; Output array address
	LDR R11,=24000000

	; Init finish

GOON
	; ն���ж�
	; R5 �������״̬
	; R6 ��ǰ״̬	��5λ��Ч
    LDR R1,=GPIOD   ; ȡ�Ƚ���״̬
    LDR R0,[R1]
	ORR R6,R0
	MOV R2,R5
	EOR R2,R6	   ;
	LDR R1,=GPIOB
	STR R2,[R1]	 

	; �Զ������ж�
	; R11 �Զ����� ����
	SUBS R11,R11,#1
	BNE GOON
    LDR R1,=GPIOD
	MOV R0,#0X128
	STR R0,[R1]
    B GOON

	; ն���ж�
    LDR R1,=GPIOD
    LDR R0,[R1]
	ORR R6,R0
	MOV R2,R5
	EOR R2,R6
	LDR R1,=GPIOB
	STR R2,[R1]

	; �����ж�
	; R8 �����ӱ�־
	; R9���������λ��  R10����������ַ
	; R0 BIT8����������    BIT9����������
	TST R8,#0
	BNE PWM_TEST
	TST R0, #BIT8
	BEQ PWM_TEST
	TST R0, #BIT9
	BEQ PUL_DIR
	ADDS R9,R9,#2
PUL_DIR		   

	; ն���ж�
    LDR R1,=GPIOD
    LDR R0,[R1]
	ORR R6,R0
	MOV R2,R5
	EOR R2,R6
	LDR R1,=GPIOB
	STR R2,[R1]

	SUBS R9,R9,#1
	BNE PUL_OUT
	MOV R9,#10
PUL_OUT
	LDR R5,[R9,R10]	   ;
	LDR R1,=GPIOB
; PWM_INIT
	MOV R7, #2400 ; 10KHz 2400
	MOV R6,#0
	LDR R1,=GPIOB
	STR R5,[R1]

	B GOON

	; PWM�ж�
	; R7 PWM ����
PWM_TEST
	SUBS R7,R7,#1
	BNE PWM_DONE  

	; ն���ж�
    LDR R1,=GPIOD   ; ȡ�Ƚ���״̬
    LDR R0,[R1]
	ORR R6,R0
	MOV R2,R5
	EOR R2,R6	   ;
	LDR R1,=GPIOB
	STR R2,[R1]

;PWM_INIT
	MOV R7, #2400 ; 10KHz 2400
	MOV R6,#0
	LDR R1,=GPIOB
	STR R5,[R1]
PWM_DONE
	B GOON	



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;RCC  ʱ������ HCLK=48MHz=HSE*6
;;;PCLK2=HCLK  PCLK1=HCLK/2
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
RCC_CONFIG_48MHZ
     LDR    R1,=0X40021000 ;RCC_CR
     LDR    R0,[R1]
     LDR    R2,=0X00010000 ;HSEON
     ORR    R0,R2
     STR    R0,[R1]
WAIT_HSE_RDY
     LDR    R2,=0X00020000 ;HSERDY
     LDR    R0,[R1]
     ANDS   R0,R2
     CMP    R0,#0
     BEQ    WAIT_HSE_RDY
     LDR    R1,=0X40022000 ;FLASH_ACR
     MOV    R0,#0X12
     STR    R0,[R1]
     LDR    R1,=0X40021004 ;RCC_CFGRʱ�����üĴ���
     LDR    R0,[R1]
 ;PLL��Ƶϵ��,PCLK2,PCLK1��Ƶ����
 ;HSE 6��ƵPCLK2=HCLK,PCLK1=HCLK/2
 ;HCLK=72MHz 0x001D0400
 ;HCLK=64MHz 0x00190400
 ;HCLK=48MHz 0x00110400
 ;HCLK=32MHz 0x00090400
 ;HCLK=24MHz 0x00050400
 ;HCLK=16MHz 0x00010400
      LDR    R2,=0x00110400
     ORR    R0,R2
     STR    R0,[R1]
     LDR    R1,=0X40021000 ;RCC_CR
     LDR    R0,[R1]
     LDR    R2,=0X01000000 ;PLLON
     ORR    R0,R2
     STR    R0,[R1]
WAIT_PLL_RDY
     LDR    R2,=0X02000000 ;PLLRDY
     LDR    R0,[R1]
     ANDS   R0,R2
     CMP    R0,#0
     BEQ    WAIT_PLL_RDY
     LDR    R1,=0X40021004 ;RCC_CFGR
     LDR    R0,[R1]
     MOV    R2,#0X02
     ORR    R0,R2
     STR    R0,[R1]
WAIT_HCLK_USEPLL
     LDR    R0,[R1]
     ANDS   R0,#0X08
     CMP    R0,#0X08
     BNE    WAIT_HCLK_USEPLL
     BX LR



;�쳣

NMI_Handler     PROC
                EXPORT  NMI_Handler                [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler          [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler          [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler           [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler                [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler           [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler             [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler            [WEAK]
                B       .
                ENDP
;�ж�
Default_Handler PROC		 
                B       .

                ENDP

                ALIGN

        END