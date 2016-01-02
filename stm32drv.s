
;常数定义---------
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

GPIOB      EQU 0X40011C00  ;GPIOB 地址
GPIOB_CRL  EQU 0X40011C00  ;低配置寄存器
GPIOB_CRH  EQU 0X40011C04  ;高配置寄存器
GPIOB_ODR  EQU 0X40011C0C  ;输出，偏移地址0Ch
GPIOB_BSRR EQU 0X40011C10  ;低置位，高清除偏移地址10h
GPIOB_BRR  EQU 0X40011C14  ;清除，偏移地址14h
IOPBEN        EQU BIT7           ;GPIOB使能位

GPIOD      EQU 0X40011C00  ;GPIOD 地址
GPIOD_CRL  EQU 0X40011C00  ;低配置寄存器
GPIOD_CRH  EQU 0X40011C04  ;高配置寄存器
GPIOD_ODR  EQU 0X40011C0C  ;输出，偏移地址0Ch
GPIOD_BSRR EQU 0X40011C10  ;低置位，高清除偏移地址10h
GPIOD_BRR  EQU 0X40011C14  ;清除，偏移地址14h
IOPDEN        EQU BIT7           ;GPIOD使能位



;GPIO寄存器地址映像
GPIOC_BASE              EQU             0x40011000
GPIOC_CRL               EQU             (GPIOC_BASE + 0x00)
GPIOC_CRH               EQU             (GPIOC_BASE + 0x04)
GPIOC_IDR               EQU             (GPIOC_BASE + 0x08)
GPIOC_ODR               EQU             (GPIOC_BASE + 0x0C)
GPIOC_BSRR              EQU             (GPIOC_BASE + 0x10)
GPIOC_BRR               EQU             (GPIOC_BASE + 0x14)
GPIOC_LCKR              EQU             (GPIOC_BASE + 0x18)
;RCC寄存器地址映像
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

;堆栈初始化
Stack_Size      EQU     0x00000400
                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp

Heap_Size       EQU     0x00000200
                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

;堆栈8字节对齐
                PRESERVE8
;使用THUMB指令
                THUMB
;复位向量
    AREA RESET,CODE,READONLY
    ;DCD STACK_TOP ;MSP主堆栈指针
    ;DCD START      ;复位，PC初始值
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
        ENTRY         ;指示开始执行
Reset_Handler		
	; Clear output
	EOR R0,R0	   ;
	LDR R1,=GPIOB
	STR R0,[R1]		   
	
	;Init begin

    ;BL.W   RCC_CONFIG_48MHZ

    LDR    R1,=RCC_APB2ENR
    LDR    R0,[R1]        ;读
    LDR    R2,=IOPBEN
    ORR    R0,R2        ;改
    STR    R0,[R1]        ;写，使能GPIOB时钟
    ;  推挽输出，50MHz
    MOV    R0,#0x33
    LDR    R1,=GPIOB_CRH ;PB.8\9 在高寄存器
    STR    R0,[R1]
    MOV    R0,#0x33333333
    LDR    R1,=GPIOB_CRL ;PB.0-7低寄存器
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
	; 斩波判断
	; R5 输出引脚状态
	; R6 当前状态	低5位有效
    LDR R1,=GPIOD   ; 取比较器状态
    LDR R0,[R1]
	ORR R6,R0
	MOV R2,R5
	EOR R2,R6	   ;
	LDR R1,=GPIOB
	STR R2,[R1]	 

	; 自动半流判断
	; R11 自动半流 计数
	SUBS R11,R11,#1
	BNE GOON
    LDR R1,=GPIOD
	MOV R0,#0X128
	STR R0,[R1]
    B GOON

	; 斩波判断
    LDR R1,=GPIOD
    LDR R0,[R1]
	ORR R6,R0
	MOV R2,R5
	EOR R2,R6
	LDR R1,=GPIOB
	STR R2,[R1]

	; 脉冲判断
	; R8 上升延标志
	; R9：输出数组位置  R10：输出数组地址
	; R0 BIT8：脉冲输入    BIT9：方向输入
	TST R8,#0
	BNE PWM_TEST
	TST R0, #BIT8
	BEQ PWM_TEST
	TST R0, #BIT9
	BEQ PUL_DIR
	ADDS R9,R9,#2
PUL_DIR		   

	; 斩波判断
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

	; PWM判断
	; R7 PWM 计数
PWM_TEST
	SUBS R7,R7,#1
	BNE PWM_DONE  

	; 斩波判断
    LDR R1,=GPIOD   ; 取比较器状态
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
;;;RCC  时钟配置 HCLK=48MHz=HSE*6
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
     LDR    R1,=0X40021004 ;RCC_CFGR时钟配置寄存器
     LDR    R0,[R1]
 ;PLL倍频系数,PCLK2,PCLK1分频设置
 ;HSE 6倍频PCLK2=HCLK,PCLK1=HCLK/2
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



;异常

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
;中断
Default_Handler PROC		 
                B       .

                ENDP

                ALIGN

        END