                save
		listing off   ; kein Listing über diesen File


		ifndef  ch552     ; verhindert Mehrfacheinbindung

ch552	equ	1

                if      (MOMCPUNAME<>"8052")
                 fatal  "Falscher Prozessortyp eingestellt: nur 8052 erlaubt!"
		endif

                if      MOMPASS=1
                 message "CH552-SFR-Definitionen"
		 message "binde \{MOMCPU}-SFRs ein"
		endif

;----------------------------------------------------------------------------
; erstmal die Sachen, die es (fast) überall gibt :

RD		BIT	P3.7		; Port 3: Schreibleitung
WR		BIT	P3.6		;         Leseleitung
T1              BIT     P3.5            ;   Testleitung 1
T0		BIT	P3.4		;	  Testleitung 0
INT1		BIT	P3.3		;         ext. Interrupt 1
INT0		BIT	P3.2		;	  ext. Interrupt 0
TXD		BIT	P3.1		;	  ser. Ausgang
RXD		BIT	P3.0		;	  ser. Eingang

TXD1		BIT	P1.3		; zweiter ser. Ausgang
RXD1		BIT	P1.2		; zweiter ser. Eingang



;---------------------------------------------------------------------------
; CH552: GENERAL REGISTERS
;---------------------------------------------------------------------------

ACC			SFRB	0E0h		; Akkumulator
B			SFRB	0F0h		; Hilfsakku für MUL/DIV
PSW			SFRB	0D0h		; Statuswort
CY			BIT	PSW.7
AC			BIT	PSW.6
F0			BIT	PSW.5
RS1			BIT	PSW.4
RS0			BIT	PSW.3
OV			BIT	PSW.2
F1			BIT	PSW.1
P			BIT	PSW.0

GLOBAL_CFG		SFR	0B1h
bBOOT_LOAD		EQU	5
bSW_RESET		EQU	4
bCODE_WE		EQU	3
bDATA_WE		EQU	2
bLDO3V3_OFF		EQU	1
bWDOG_EN		EQU	0

CHIP_ID			SFR	0A1h
SAFE_MOD		SFR	0A1h

DPH			SFR	83h		; Datenzeiger Bit 8..15
DPL			SFR	82h		; Datenzeiger Bit 0..7
DPTR			SFR	82h
DPS			SFR	0A2h		; Bit 0=Umschaltung DPTR0<-->DPTR1

SP			SFR	81h		; Stapelzeiger


;---------------------------------------------------------------------------
; CH552: SYSTEM CLOCK CONFIGURATION REGISTER
;---------------------------------------------------------------------------

CLOCK_CFG		SFR	0B9h
bOSC_EN_INT		EQU	7
bOSC_EN_XT		EQU	6
bWDOG_IF_TO		EQU	5
bROM_CLK_FAST		EQU	4
bRST			EQU	3
bMASK_SYS_CK_SEL2	EQU	2
bMASK_SYS_CK_SEL1	EQU	1
bMASK_SYS_CK_SEL0	EQU	0


;---------------------------------------------------------------------------
; CH552: INTERRUPTS
;---------------------------------------------------------------------------

IP_EX			SFR	0E9h
bIP_LEVEL		EQU	7
bIP_GPIO		EQU	6
bIP_PWMX		EQU	5
bIP_UART1		EQU	4
bIP_ADC			EQU	3
bIP_USB			EQU	2
bIP_TKEY		EQU	1
bIP_SPI0		EQU	0

IE_EX			SFRB	0E8h
IE_WDOG			BIT	IE_EX.7
IE_GPIO			BIT	IE_EX.6
IE_PWMX			BIT	IE_EX.5
IE_UART1		BIT	IE_EX.4
IE_ADC			BIT	IE_EX.3
IE_USB			BIT	IE_EX.2
IE_TKEY			BIT	IE_EX.1
IE_SPI0			BIT	IE_EX.0

GPIO_IE			SFR	0C7h
bIE_IO_EDGE		EQU	7
bIE_RXD1_LO		EQU	6
bIE_P1_5_LO		EQU	5
bIE_P1_4_LO		EQU	4
bIE_P1_3_LO		EQU	3
bIE_RST_HI		EQU	2
bIE_P3_1_LO		EQU	1
bIE_RXD0_LO		EQU	0

IP			SFRB	0B8h		; Interruptprioritäten
PH_FLAG			BIT	IP.7
PL_FLAG			BIT	IP.6
PT2			BIT	IP.5		; Priorität Interrupt Timer 2
PS			BIT	IP.4		; Priorität serieller Interrupt
PT1			BIT	IP.3		; Priorität Interrupt Timer 1
PX1			BIT	IP.2		; Priorität externer Interrupt 1
PT0			BIT	IP.1		; Priorität Interrupt Timer 0
PX0			BIT	IP.0		; Priorität externer Interrupt 0

IE			SFRB	0A8h		; Interruptfreigaben
EA			BIT	IE.7		; generelle Interruptfreigabe
E_DIS			BIT	IE.6		; generelle Interruptsperre
ET2			BIT	IE.5		; Interrupt Timer 2 freigeben
ES			BIT	IE.4		; Interrupts ser. Schnittstelle freigeben
ET1			BIT	IE.3		; Interrupt Timer 1 freigeben
EX1			BIT	IE.2		; externen Interrupt 1 freigeben
ET0			BIT	IE.1		; Interrupt Timer 0 freigeben
EX0			BIT	IE.0		; externen Interrupt 0 freigeben


;---------------------------------------------------------------------------
; CH552: GPIO
;---------------------------------------------------------------------------

P1			SFRB	90h
P1_MOD_OC		SFR	92h
P1_DIR_PU		SFR	93h
P2			SFRB	0A0h
P3			SFRB	0B0h
P3_MOD_OC		SFR	96h
P3_DIR_PU		SFR	97h
PIN_FUNC		SFR	0C6h
XBUS_AUX		SFR	0A2h


;---------------------------------------------------------------------------
; CH552: SPI
;---------------------------------------------------------------------------

SPI0_SETUP		SFR	0FCh
SPI0_S_PRE		SFR	0FBh
SPI0_CK_SE		SFR	0FBh
SPI0_CTRL		SFR	0FAh
SPI0_DATA		SFR	0F9h
SPI0_STAT		SFRB	0F8h
S0_FST_ACT		BIT	SPI0_STAT.7
S0_IF_OV		BIT	SPI0_STAT.6
S0_IF_FIRST		BIT	SPI0_STAT.5
S0_IF_BYTE		BIT	SPI0_STAT.4
S0_FREE			BIT	SPI0_STAT.3
S0_T_FIFO		BIT	SPI0_STAT.2
S0_R_FIFO		BIT	SPI0_STAT.0


;---------------------------------------------------------------------------
; CH552: POWER AND SLEEP CONTROL REGISTERS
;---------------------------------------------------------------------------

WDOG_COUNT		SFR	0FFh
RESET_KEEP		SFR	0FEh
WAKE_CTRL		SFR	0A9h
bWAK_BY_USB		EQU	7
bWAK_RXD1_LO		EQU	6
bWAK_P1_5_LO		EQU	5
bWAK_P1_4_LO		EQU	4
bWAK_P1_3_LO		EQU	3
bWAK_RST_HI		EQU	2
bWAK_P3_2E_3L		EQU	1
bWAK_RXD0_LO		EQU	0
PCON			SFR	87h		; "Power-Management"
bSMOD			EQU	7
bRST_FLAG1		EQU	5
bRST_FLAG0		EQU	4
bGF1			EQU	3
bGF0			EQU	2
bPD			EQU	1


;---------------------------------------------------------------------------
; CH552: ADC
;---------------------------------------------------------------------------

ADC_CTRL		SFRB	80h
ADC_CFG			SFR	9Ah
ADC_DATA		SFR	9Fh
CMPO			BIT	ADC_CTRL.7
CMP_IF			BIT	ADC_CTRL.6
ADC_IF			BIT	ADC_CTRL.5
ADC_START		BIT	ADC_CTRL.4
CMP_CHAN		BIT	ADC_CTRL.3
ADC_CHAN1		BIT	ADC_CTRL.1
ADC_CHAN0		BIT	ADC_CTRL.0


;---------------------------------------------------------------------------
; CH552: TOUCH-KEY
;---------------------------------------------------------------------------

TKEY_CTRL		SFR	0C3h
TKEY_DATH		SFR	0C5h
TKEY_DATL		SFR	0C4h
TKEY_DAT		SFR	0C4h

bTKC_IF			EQU	7
bTKC_2MS		EQU	4
bTKC_CHAN2		EQU	2
bTKC_CHAN1		EQU	1
bTKC_CHAN0		EQU	0


;---------------------------------------------------------------------------
; CH552: TIMER
;---------------------------------------------------------------------------

TH0			SFR	8Ch		; Daten Timer 0
TL0			SFR	8Ah
TH1			SFR	8Dh		; Daten Timer 1
TL1			SFR	8Bh
TMOD			SFR	89h		; Timer 0/1 Betriebsartenregister
TCON			SFRB	88h		; Timer 0/1 Steuerregister
TF1			BIT	TCON.7		; Überlauf Timer 1
TR1			BIT	TCON.6		; Timer 1 laufenlassen
TF0			BIT	TCON.5		; Überlauf Timer 0
TR0			BIT	TCON.4		; Timer 0 laufenlassen
IE1			BIT	TCON.3		; Externer Interrupt 1 aufgetreten
IT1			BIT	TCON.2		; Einstellung Triggerflanke externer Interrupt 1
IE0			BIT	TCON.1		; Externer Interrupt 0 aufgetreten
IT0			BIT	TCON.0		; Einstellung Triggerflanke externer Interrupt 0

bT1_GATE		EQU	7
bT1_CT			EQU	6
bT1_M1			EQU	5
bT1_M0			EQU	4
bT0_GATE		EQU	3
bT0_CT			EQU	2
bT0_M1			EQU	1
bT0_M0			EQU	0

TH2			SFR	0CDh		; Daten Timer 2
TL2			SFR	0CCh
T2COUNT			SFR	0CCh
T2CAP1H			SFR	0CFh
T2CAP1L			SFR	0CEh
T2CAP1			SFR	0CEh
RCAP2H			SFR	0CBh		; Capture-Register
RCAP2L			SFR	0CAh
RCAP2			SFR	0CAh
T2MOD			SFR	0C9h		; Timer 2 Betriebsartenregister
T2CON			SFRB	0C8h		; Steuerregister Timer 2
TF2			BIT	T2CON.7		; Overflow Timer 2
CAP1F			BIT	T2CON.7
EXF2			BIT	T2CON.6		; Reload aufgetreten
RCLK			BIT	T2CON.5		; Timer 2 liefert RxD-Takt
TCLK			BIT	T2CON.4		; Timer 2 liefert TxD-Takt
EXEN2			BIT	T2CON.3		; externer Enable Timer 2
TR2			BIT	T2CON.2		; Timer 2 starten
C_T2			BIT	T2CON.1		; Timer 2 als Zähler
CP_RL2			BIT	T2CON.0		; Capture erlauben

bTMR_CLK		EQU	7
bT2_CLK			EQU	6
bT1_CLK			EQU	5
bT0_CLK			EQU	4
bT2_CAP_M1		EQU	3
bT2_CAP_M0		EQU	2
bT2OE			EQU	1
bT2_CAP1_EN		EQU	0

;---------------------------------------------------------------------------
; CH552: USB
;---------------------------------------------------------------------------

USB_C_CTRL		SFR	091h		; USB type-C control register
USB_CTRL		SFR	0E2h		; USB control register
USB_INT_FG		SFR	0D8h
USB_INT_ST		SFR	0D9h
USB_MIS_ST		SFR	0DAh
USB_RX_LEN		SFR	0DBh
USB_INT_EN		SFR	0E1h		; USB interrupt enable register
USB_DEV_AD		SFR	0E3h		; USB device address register

UDEV_CTRL		SFR	0D1h		; USB device physical port control register
UEP4_1_MOD		SFR	0EAh		; Endpoint 1, 4 mode control register
UEP2_3_MOD		SFR	0EBh		; Endpoint 2, 3 mode control register


UEP0_CTRL		SFR	0DCh		; Endpoint 0 control register
UEP0_T_LEN		SFR	0DDh		; Endpoint 0 send length register
UEP0_DMA_H		SFR	0EDh		; Endpoint 0 buffer start address high byte
UEP0_DMA_L		SFR	0ECh		; Endpoint 0 buffer start address low byte
UEP0_DMA		SFR	0ECh

UEP1_CTRL		SFR	0D2h		; Endpoint 1 control register
UEP1_T_LEN		SFR	0D3h		; Endpoint 1 send length register
UEP1_DMA_H		SFR	0EFh		; Endpoint 1 buffer start address high byte
UEP1_DMA_L		SFR	0EEh		; Endpoint 1 buffer start address low byte
UEP1_DMA		SFR	0EEh


UIF_IS_NAK		BIT	USB_INT_FG.7
UIF_TOG_OK		BIT	USB_INT_FG.6
UIF_SIE_FREE		BIT	USB_INT_FG.5
UIF_FIFO_OV		BIT	USB_INT_FG.4
UIF_HST_SOF		BIT	USB_INT_FG.3
UIF_SUSPEND		BIT	USB_INT_FG.2
UIF_TRANSFER		BIT	USB_INT_FG.1
UIF_BUS_RST		BIT	USB_INT_FG.0

; bUEP_R_RES1 & bUEP_R_RES0: handshake response type for USB endpoint X receiving (OUT)
UEP_R_RES_ACK		EQU	00h		; 00: ACK (ready)
UEP_R_RES_TOUT		EQU	04h		; 01: no response, time out to host, for non-zero endpoint isochronous transactions
UEP_R_RES_NAK		EQU	08h		; 10: NAK (busy)
UEP_R_RES_STALL		EQU	0Ch		; 11: STALL (error)

; bUEP_T_RES1 & bUEP_T_RES0: handshake response type for USB endpoint X transmittal (IN)
UEP_T_RES_ACK		EQU	00h		; 00: DATA0 or DATA1 then expecting ACK (ready)
UEP_T_RES_TOUT		EQU	01h		; 01: DATA0 or DATA1 then expecting no response, time out from host, for non-zero endpoint isochronous transactions
UEP_T_RES_NAK		EQU	02h		; 10: NAK (busy)
UEP_T_RES_STALL		EQU	03h		; 11: STALL (error)


;---------------------------------------------------------------------------
; CH552: FLASH PROGRAMMING
;---------------------------------------------------------------------------

ROM_DATA_H		SFR	08Fh
ROM_DATA_L		SFR	08Eh
ROM_DATA		SFR	08Eh
ROM_STATUS		SFR	086h
ROM_CTRL		SFR	086h
ROM_ADDR_H		SFR	085h
ROM_ADDR_L		SFR	084h
ROM_ADDR		SFR	084h

bROM_ADDR_OK		EQU	6
bROM_CMD_ERR		EQU	1

;---------------------------------------------------------------------------
; CH552: VECTOR TABLE
;---------------------------------------------------------------------------

INT_ADDR_RST		EQU	0000h
INT_ADDR_INT0		EQU	0003h
INT_ADDR_TMR0		EQU	000Bh
INT_ADDR_INT1		EQU	0013h
INT_ADDR_TMR1		EQU	001Bh
INT_ADDR_UART0		EQU	0023h
INT_ADDR_TMR2		EQU	002Bh
INT_ADDR_SPI0		EQU	0033h
INT_ADDR_TKEY		EQU	003Bh
INT_ADDR_USB		EQU	0043h
INT_ADDR_ADC		EQU	004Bh
INT_ADDR_UART1		EQU	0053h
INT_ADDR_PWMX		EQU	005Bh
INT_ADDR_GPIO		EQU	0063h
INT_ADDR_WDOG		EQU	006Bh


;---------------------------------------------------------------------------
;---------------------------------------------------------------------------
;---------------------------------------------------------------------------

SCON		SFRB	98h		; ser. Schnittstelle: Steuerregister

SM0		BIT     SCON.7          ; Betriebsarten
SM1		BIT     SCON.6
SM2		BIT     SCON.5
REN		BIT	SCON.4		; Empfangssperre
TB8		BIT	SCON.3		; 9. zu sendendes Bit
RB8		BIT	SCON.2		; 9. empfangenes Bit
TI		BIT	SCON.1		; Senderinterruptflag
RI		BIT	SCON.0		; Empfängerinterruptflag
SBUF		SFR	99h		;  "         "        Datenregister

SBAUD1		SFR	0C2h



SCON0		SFR	SCON 		; Steuerregister
SM0_0		BIT	SCON0.7
SM1_0		BIT	SCON0.6
SM2_0 		BIT	SCON0.5
REN_0		BIT	SCON0.4
TB8_0		BIT	SCON0.3
RB8_0		BIT	SCON0.2
TI_0		BIT	SCON0.1
RI_0		BIT	SCON0.0
SBUF0		SFR	SBUF
SCON1		SFRB	0c0h		; Register zweite serielle Schnittstelle
SM0_1		BIT	SCON1.7
SM1_1		BIT	SCON1.6
SM2_1 		BIT	SCON1.5
REN_1		BIT	SCON1.4
TB8_1		BIT	SCON1.3
RB8_1		BIT	SCON1.2
U1TI		BIT	SCON1.1
U1RI		BIT	SCON1.0
SBUF1           SFR	0c1h            ; Datenregister





;----------------------------------------------------------------------------
; Da man die Register beim 8051 nicht direkt pushen und poppen kann, muss
; man dies über direkte Adressen tun. Da dazu die Kenntnis der gewählten
; Registerbank erforderlich ist, steht hier das Makro USING zur Verfügung,
; welches in den Symbolen AR0..AR7 die momentanen Registeradressen ablegt.
; USING erwartet als Parameter die gewählte Banknummer.

Bank0		equ	0		; nur der Schönheit halber...
Bank1		equ	1
Bank2		equ	2
Bank3		equ	3

using		macro	bank
		if	(bank<0)||(bank>3)          ; nur 0..3 erlaubt
                 error  "Falsche Banknummer: \{BANK}"
                endif

                ifdef	RegUsage	; Buchführung über benutzte Bänke
RegUsage	 set	RegUsage|(2^bank)
		elseif
RegUsage	 set	2^bank
		endif

ar0		set	bank*8		; Symbole besetzen
ar1		set	ar0+1
ar2		set	ar0+2
ar3		set	ar0+3
ar4		set	ar0+4
ar5		set	ar0+5
ar6		set	ar0+6
ar7		set	ar0+7
		endm

                endif

                restore                 ; wieder erlauben

