;----------------------------------------------------------------
;
; Firmware für "XLink CR"
;
;
; Copyright 12.2022 by Thomas Tahsin-Bey
;
;
; use Alfred Arnolds AS to assemble this source!
;
; asw XLinkCR.asm -L
; p2bin XLinkCR
;
;
; V0.5		first official working release
;
;----------------------------------------------------------------

			CPU 8052
			INCLUDE ch552

;
; eigene Funktionen definieren
;
hi			FUNCTION x,(x>>8)&255
lo			FUNCTION x,x&255

;
; ASCII-Konstanten
;
CR			EQU	00Dh					;Ascii-Code für <CARRIAGE RETURN>
LF			EQU	00Ah					;Ascii-Code für <LINE FEED>
SPACE			EQU	020h					;Ascii-Code für's Leerzeichen

;
; Bit-Definitionen
;

bReqError		EQU	010h					;Bit 0 in Adresse 022h
bXlink_Command		EQU	011h					;Bit 1 in Adresse 022h
bTemp			EQU	012h					;Bit 2 in Adresse 022h
bXlinkMode_Send		EQU	018h					;Bit 0 in Adresse 023h
bXlinkMode_Receive	EQU	019h					;Bit 1 in Adresse 023h
bLastAck		EQU	01Ah

;
; Byte-Definitionen
;

bRequest		EQU	031h
bUSB_Setup_Cmd		EQU	032h
bLength			EQU	033h

wValue			EQU	034h
wValueLo		EQU	034h
wValueHi		EQU	035h

wRequest_Length		EQU	036h
wRequest_LengthLo	EQU	036h
wRequest_LengthHi	EQU	037h

Descriptor_Address_HI	EQU	038h
Descriptor_Address_LO	EQU	039h

my_memcpy_SRC_HI	EQU	03Ch
my_memcpy_SRC_LO	EQU	03Dh

XLink_TransferLength_HI	EQU	048h
XLink_TransferLength_LO	EQU	049h

Counter			EQU	04Ch
Length			EQU	04Dh
Temp			EQU	04Eh
Temp2			EQU	04Fh


Stack			EQU	060h					;hier beginnt im RAM der Stack (32 Bytes (bis 7Fh))

EP0_Length		EQU	64					;Länge des EP0: 64 Bytes
EP0_Buffer_Base		EQU	0000h					;EP0-Basisadresse

;
; Byte-Definitionen
;

Pin_Reset		BIT	P3.2					;'Host CPU'-Reset ist auf Pin P3.2
Pin_Strobe		BIT	P3.5					;Strobe ist auf Pin P3.5
Pin_Acknowledge		BIT	P3.4					;Die Rückmeldung des Host-Systems ist auf Pin P3.4

;----------------------------------------------------------------

			ORG	INT_ADDR_RST				;Einsprung bei Reset
			jmp	Coldstart
			ORG	INT_ADDR_USB				;Einsprung bei USB-Interrupt
			jmp	USB_Interrupt_Entry


			ORG	0080h					;Programmbeginn ab hier...
Coldstart:		mov	SP,#Stack				;Stack initialisieren

			clr	A
			mov	PSW,A
			mov	TCON,A
			mov	IE,A
			mov	SCON,A
			mov	XBUS_AUX,A

			mov	SAFE_MOD,#055h
			mov	SAFE_MOD,#0AAh
			mov	CLOCK_CFG,#084h				;Clock auf 12MHz setzen
			mov	SAFE_MOD,#000h

Init_Control_Lines:	mov	P3,#0FFh				;P3.2:	Reset:		Ausgang / OpenDrain No Pullup / Init: High-Pegel
			mov	P3_DIR_PU,#0FBh				;P3.4:	Acknowledge:	Eingang / Pullup (Standard 8051)
			mov	P3_MOD_OC,#0DFh				;P3.5:	Strobe:		Ausgang / PushPull / Init: High-Pegel

Init_Transfer_Port:	mov	P1,#0FFh				;Transfer-Port P1 initialisieren
			mov	P1_DIR_PU,#0FFh
			mov	P1_MOD_OC,#0FFh				;Port P1 auf Standard-8051-Ausgang schalten ("0=Drive, 1=PullUp"). P1_MOD_OC=0 setzt Port auf Push/Pull-Mode

Init_Variables:		clr	bXlink_Command
			clr	bReqError

			mov	C,Pin_Acknowledge			;Pegel vom Acknowledge-Bit holen
			mov	bLastAck,C

			mov	bUSB_Setup_Cmd,#000h

Init_Timer_and_UART:	clr	TR0
			clr	TR1
			mov	PIN_FUNC,#080h				;Pins für die Schnittstelle: P3.0/P3.1
			mov	SCON0,#050h				;REN bleibt gelöscht: nur TX, kein RX		Baud	CPU	T2MOD	SMOD	TH1
			mov	T2MOD,#0A0h				;A0h: Timer-Clock <== System-Clock  (1:1)	2400	12MHz	0A0h	0	064h
			mov	TMOD,#021h				;Timer setzen (T1 8-Bit, selbst nachladend)	9600	12MHz	0A0h	1	0B2h
			mov	PCON,#080h				;doppelte Geschwindigkeit für UART		19200	12MHz	0A0h	1	0D9h
			mov	TH1,#0F3h				;Baudrate: 57600 Baud				57600	12MHz	0A0h	1	0F3h
			setb	TR1					;Timer starten					115200	24MHz	0A0h	1	0F3h
			setb	TI


Init_USB:		clr	A
			mov	USB_C_CTRL,A
			mov	USB_CTRL,#006h				;Reset USB and FIFO durch Setzen der entsprechenden Bits im Register
			nop
			mov	USB_CTRL,A				;Reset-Bit löschen
			mov	UEP2_3_MOD,A				;nur Endpoint 0 soll aktiv sein
			mov	UEP4_1_MOD,A				;--> single 64 bytes buffer for endpoint 0
			mov	UEP0_DMA_H,#hi(EP0_Buffer_Base)		;Adresse für DMA-Transfer setzen (Hinweis: Buffer liegt im XRAM)
			mov	UEP0_DMA_L,#lo(EP0_Buffer_Base)
			mov	UEP0_CTRL,#UEP_R_RES_ACK|UEP_T_RES_NAK	;folgende OUT-/SETUP-Pakete mit ACK beantworten, folgende IN-Pakete mit NAK beantworten
			mov	USB_DEV_AD,A
			mov	UDEV_CTRL,#080h
			mov	USB_CTRL,#029h				;High-Speed (12Mbps)
			mov	UDEV_CTRL,#081h				;Hardware-Port vom USB-Device einschalten
			mov	USB_INT_FG,#0FFh			;alle USB-Interrupt-Flags löschen
			mov	USB_INT_EN,#007h			;mögliche IRQs: Suspend, Transfer, Bus_Reset

			call	Send_Inline_Text			;DEBUG: "CPU-Reset"-Message ausgeben
			db	CR,"CPU: Reset, Coldstart",CR|80h


Enable_Interrupts:	mov	IE_EX,#004h				;Interrupts von USB zulassen
			mov	IE,#080h

MainMain:	

			jmp	MainMain

			mov	A,#"0"
			call	Send_Char
			call	Delay
			mov	A,#"1"
			call	Send_Char
			call	Delay
			mov	A,#"2"
			call	Send_Char
			call	Delay
			mov	A,#"3"
			call	Send_Char
			call	Delay
			jmp	MainMain

Delay:			mov	r4,#010h
-			mov	r3,#0FFh
-			mov	r2,#0FFh
-			djnz	r2,-
			djnz	r3,--
			djnz	r4,---
			ret


;----------------------------------------------------------------
Send_Char:		jnb	TI,$					;warte, bis letzte Übertragung abgeschlossen ist
			clr	TI
			mov	SBUF,A					;Senden des Zeichens im Akku
			cjne	A,#CR,.exit
			mov	A,#LF					;Senden von "LF", da vorheriges Byte ein "CR" war
			jmp	Send_Char
.exit:			ret


Send_CRLF:		mov	A,#CR
			jmp	Send_Char


Send_Hex:		push	ACC					;Sendet Akku hexadezimal an den Host
			swap	A
			call	Send_Nibble
			pop	ACC
Send_Nibble:		anl	A,#00Fh					;rechtes Nibble isolieren
			clr	C
			subb	A,#00Ah					;Nibble - 10
			jc	+					;es ist eine Zahl?, dann springen
			add	A,#007h					;sonst Korrekturwert addieren
+			add	A,#03Ah					;und nach ASCII wandeln
			jmp	Send_Char				;Zeichen ausgeben (und RET)


Send_Text:		clr	A
			movc	A,@A+DPTR
			jz	.exit
			call	Send_Char
			inc	DPTR
			jmp	Send_Text
.exit:			ret


Send_XMEM_EP0:		ret
			
			mov	DPTR,#EP0_Buffer_Base+000h
Send_XMEM_EP0_Loop:	mov	A,#SPACE
			call	Send_Char
			movx	A,@DPTR
			call	Send_Hex
			inc	DPTR
			djnz	Counter,Send_XMEM_EP0_Loop
			jmp	Send_CRLF


Send_Inline_Text:	pop	DPH
			pop	DPL
.loop:			clr	A
			movc	A,@A+DPTR
			inc	DPTR
			jbc	ACC.7,.exit
			call	Send_Char
			jmp	.loop
.exit:			call	Send_Char
			clr	A
			jmp	@A+DPTR					;Nach dem Textende im Programm weitermachen...



Send_Strobe:		clr	TR0					;Strobe senden
			mov	TL0,#lo(10000h-5)
			mov	TH0,#hi(10000h-5)			;Strobe-Länge: 5µs
			clr	TF0
			clr	Pin_Strobe
			setb	TR0
.Wait:			jnb	TF0,.Wait
			setb	Pin_Strobe
			clr	TR0
			ret


;----------------------------------------------------------------
USB_Interrupt_Entry:	push	ACC
			push	DPH
			push	DPL
			push	PSW					;Akku retten

USB_Irq_Test_Reset:	jnb	UIF_BUS_RST,USB_Irq_Test_Suspend	;Wurde der Interrupt durch einen Bus-Reset ausgelöst? Nein, dann springen
			mov	UEP0_CTRL,#UEP_R_RES_ACK|UEP_T_RES_NAK	;folgende OUT-/SETUP-Pakete mit ACK beantworten, folgende IN-Pakete mit NAK beantworten
			clr	A
			mov	USB_DEV_AD,A				;Device-Adresse auf 0 setzen

;			call	Send_Inline_Text			;DEBUG: "USB-Reset"-Message ausgeben
;			db	CR,"USB: Reset",CR|80h

			clr	UIF_BUS_RST				;Löschen des Interrupt-Flags für "USB Bus Reset"
			pop	PSW
			pop	DPL
			pop	DPH
			pop	ACC
			reti


USB_Irq_Test_Suspend:	jnb	UIF_SUSPEND,USB_Irq_Test_Transfer	;Wurde der Interrupt durch einen Suspend ausgelöst? Nein, dann springen

;			call	Send_Inline_Text			;DEBUG: "USB-Suspend"-Message ausgeben
;			db	CR,"USB: Suspend",CR|80h

			clr	UIF_SUSPEND				;Löschen des Interrupt-Flags für "USB Suspend"
			pop	PSW
			pop	DPL
			pop	DPH
			pop	ACC
			reti


USB_Irq_Test_Transfer:	jb	UIF_TRANSFER,USB_Check_Transfer		;Wurde der Interrupt durch einen fertigen USB-Transfer ausgelöst? Ja, dann springen
USB_Irq_Done:		mov	USB_INT_FG,#0FFh			;alle USB-Interrupt-Flags löschen
			pop	PSW
			pop	DPL
			pop	DPH
			pop	ACC
			reti


USB_Check_Transfer:	mov	A,USB_INT_ST				;Transfer-Status lesen
			anl	A,#03Fh					;Token und Endpoint maskieren
			cjne	A,#000h,+
			jmp	USB_Transfer_is_OUT
+			cjne	A,#020h,+
			jmp	USB_Transfer_is_IN
+			cjne	A,#030h,+
			jmp	USB_Transfer_is_SETUP

+			jmp	USB_Transfer_done



USB_Transfer_is_SETUP:	mov	A,USB_RX_LEN				;Länge der eingehenden Daten holen
			xrl	A,#8					;mit 8 vergleichen (jedes Setup-Paket ist 8 Bytes lang)
			jnz	USB_Setup_Error				;Länge des Datenpakets ist ungleich 8, also springen

			clr	bReqError				;gültiges Setup-Paket ==> Fehler-Flag löschen

			mov	DPTR,#EP0_Buffer_Base+0			;bmRequestType lesen
			movx	A,@DPTR
			anl	A,#0E0h
			mov	bUSB_Setup_Cmd,A
			inc	DPTR
			movx	A,@DPTR
			mov	bRequest,A
			anl	A,#00Fh
			orl	A,#010h
			orl	bUSB_Setup_Cmd,A			;RequestType und Request kombiniert (dabei ist Bit 4 immer gesetzt!)

			mov	DPTR,#EP0_Buffer_Base+6			;wLength aus Buffer lesen
			movx	A,@DPTR
			mov	wRequest_LengthLo,A
			inc	DPTR
			movx	A,@DPTR
			mov	wRequest_LengthHi,A


USB_Setup_Dispatcher:	mov	XBUS_AUX,#000h
			mov	DPTR,#USB_Setup_ChkReq_Table
USB_Setup_Dispatcher_1:	clr	A
			movc	A,@A+DPTR
			jz	USB_Setup_Error
			xrl	A,bUSB_Setup_Cmd
			inc	DPTR
			jz	USB_Setup_Dispatcher_2
			inc	DPTR
			inc	DPTR
			jmp	USB_Setup_Dispatcher_1
USB_Setup_Dispatcher_2:	movc	A,@A+DPTR
			push	ACC
			inc	DPTR
			clr	A
			movc	A,@A+DPTR
			mov	DPH,A
			pop	DPL
			clr	A
			jmp	@A+DPTR
			
USB_Setup_ChkReq_Table:	DB	015h,lo(USB_STD_Set_Address),hi(USB_STD_Set_Address)
			DB	019h,lo(USB_STD_Set_Config),hi(USB_STD_Set_Config)
			DB	090h,lo(USB_STD_Get_Status),hi(USB_STD_Get_Status)
			DB	096h,lo(USB_STD_Get_Descriptor),hi(USB_STD_Get_Descriptor)
			DB	098h,lo(USB_STD_Get_Config),hi(USB_STD_Get_Config)
			DB	051h,lo(USB_SETUP_XLK_Reset),hi(USB_SETUP_XLK_Reset)
			DB	052h,lo(USB_SETUP_XLK_Strobe),hi(USB_SETUP_XLK_Strobe)
			DB	054h,lo(USB_SETUP_XLK_Input),hi(USB_SETUP_XLK_Input)
			DB	055h,lo(USB_SETUP_XLK_Output),hi(USB_SETUP_XLK_Output)
			DB	057h,lo(USB_SETUP_XLK_Write),hi(USB_SETUP_XLK_Write)
			DB	058h,lo(USB_SETUP_XLK_Send),hi(USB_SETUP_XLK_Send)
			DB	0D3h,lo(USB_SETUP_XLK_Ack),hi(USB_SETUP_XLK_Ack)
			DB	0D6h,lo(USB_SETUP_XLK_Read),hi(USB_SETUP_XLK_Read)
			DB	0D9h,lo(USB_SETUP_XLK_Receive),hi(USB_SETUP_XLK_Receive)
			DB	0


USB_Setup_Error:	setb	bReqError				;Bit für Request-Error setzen
			jmp	USB_Setup_Break				;und Setup-Befehl abbrechen


USB_Setup_Break:	jnb	bReqError,USB_Next_Packet
			mov	bRequest,#0FFh
			mov	bUSB_Setup_Cmd,#0			;gespeichertes Setup-Kommando löschen
			mov	UEP0_CTRL,#0CFh				;#bUEP_R_TOG or bUEP_T_TOG or UEP_R_RES_STALL or UEP_T_RES_STALL; 0xCF ;
			jmp	USB_Transfer_done


USB_Next_Packet:	mov	A,wRequest_LengthLo			;Prüfen, ob Antwort in bLength länger ist als angeforderte maximale Länge
			setb	C
			subb	A,bLength
			mov	A,wRequest_LengthHi
			subb	A,#0
			jc	USB_Next_Packet2			;springen, wenn Antwort in bLength länger ist. Dann nur maximale Länge übertragen
			mov	wRequest_LengthLo,bLength		;sonst Request_Length auf bLength begrenzen
			mov	wRequest_LengthHi,#0
USB_Next_Packet2:	call	USB_Irq_PrepDescPacket

			clr	P3.0
			nop
			nop
			setb	P3.0

			mov	UEP0_CTRL,#0C0h				;#bUEP_R_TOG or bUEP_T_TOG	;0xC0
;			mov	UEP0_CTRL,#UEP_R_TOG | UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK
			jmp	USB_Transfer_done


USB_Transfer_done:	clr	UIF_TRANSFER				;Löschen des Interrupt-Flags für "USB Transfer"
			pop	PSW
			pop	DPL
			pop	DPH
			pop	ACC
			reti




USB_STD_Get_Descriptor:	mov	DPTR,#EP0_Buffer_Base+3
			movx	A,@DPTR
USB_Standard_Request_5:	cjne	A,#1,USB_Standard_Request_6
USB_Irq_GetDeviceDesc:	mov	Descriptor_Address_HI,#hi(Descriptor_Device)
			mov	Descriptor_Address_LO,#lo(Descriptor_Device)
			mov	bLength,#Descriptor_Device_Len
			jmp	USB_Setup_Break
USB_Standard_Request_6:	cjne	A,#2,USB_Standard_Request_7
USB_Irq_GetConfigDesc:	mov	Descriptor_Address_HI,#hi(Descriptor_Config)
			mov	Descriptor_Address_LO,#lo(Descriptor_Config)
			mov	bLength,#012h
			jmp	USB_Setup_Break
USB_Standard_Request_7:	cjne	A,#3,USB_Descriptor_Unknown
USB_Irq_GetStringDesc:	mov	DPTR,#EP0_Buffer_Base+2			;prüfen, was für ein String-Descriptor angefordert wurde
			movx	A,@DPTR
USB_Irq_GetStringDesc0:	cjne	A,#0,USB_Irq_GetStringDesc1
			mov	Descriptor_Address_HI,#hi(Descriptor_String0)
			mov	Descriptor_Address_LO,#lo(Descriptor_String0)
			mov	bLength,#Descriptor_String0_Len
			jmp	USB_Setup_Break
USB_Irq_GetStringDesc1:	cjne	A,#1,USB_Irq_GetStringDesc2
			mov	Descriptor_Address_HI,#hi(Descriptor_String1)
			mov	Descriptor_Address_LO,#lo(Descriptor_String1)
			mov	bLength,#Descriptor_String1_Len
			jmp	USB_Setup_Break
USB_Irq_GetStringDesc2:	cjne	A,#2,USB_Irq_GetStringDesc3
			mov	Descriptor_Address_HI,#hi(Descriptor_String2)
			mov	Descriptor_Address_LO,#lo(Descriptor_String2)
			mov	bLength,#Descriptor_String2_Len
			jmp	USB_Setup_Break
USB_Irq_GetStringDesc3:	cjne	A,#3,USB_Irq_GetStringNull
			mov	Descriptor_Address_HI,#hi(Descriptor_String3)
			mov	Descriptor_Address_LO,#lo(Descriptor_String3)
			mov	bLength,#Descriptor_String3_Len
			jmp	USB_Setup_Break
USB_Irq_GetStringNull:	mov	Descriptor_Address_HI,#hi(Descriptor_Null)
			mov	Descriptor_Address_LO,#lo(Descriptor_Null)
			mov	bLength,#002h
			jmp	USB_Setup_Break


USB_Descriptor_Unknown:	setb	bReqError
			jmp	USB_Setup_Break
			

USB_STD_Set_Address:	mov	DPTR,#EP0_Buffer_Base+002h		;neue Adresse für dieses USB-Device laden
			movx	A,@DPTR
			mov	wValueLo,A				;zwischenspeichern in wValueLo
			jmp	USB_Setup_Break


USB_STD_Get_Status:	mov	DPTR,#EP0_Buffer_Base+000h
			mov	A,wValueLo
			movx	@DPTR,A
			mov	bLength,#0				;Länge der Antwort setzen
			jmp	USB_Setup_Break


USB_STD_Get_Config:	mov	DPTR,#EP0_Buffer_Base+000h
			mov	A,wValueLo
			movx	@DPTR,A
			mov	bLength,#1				;Länge der Antwort setzen
			jmp	USB_Setup_Break


;-----------------------------------------------------------------------
USB_STD_Set_Config:	mov	DPTR,#EP0_Buffer_Base+002h		;überflüssig?
			movx	A,@DPTR
			mov	wValueLo,A				;geforderte Konfiguration in wValueLo zwischenspeichern (hier nicht relevant)
			jmp	USB_Setup_Break


;-----------------------------------------------------------------------
USB_SETUP_XLK_Reset:	clr	TR0					;(1) Reset-Signal zum Host senden
			mov	TL0,#lo(10000h-10000)			;10000µs warten (=> 65536d-10000d=55536d=D8F0h)
			mov	TH0,#hi(10000h-10000)
			clr	TF0					;Overflow-Flag löschen
			clr	Pin_Reset				;Reset-Leitung auf Low ziehen (geht auf /RES vom Userport)
			setb	TR0					;Timer0 für die Verzögerung von 1000µs starten
.wait:			jnb	TF0,.wait				;warten, bis Timer0 überläuft
			setb	Pin_Reset				;Reset-Leitung wieder auf High legen
			clr	TR0					;Timer wieder stoppen

			mov	bLength,#0				;Länge der Antwort setzen
			jmp	USB_Xlink_Break				;und Routine beenden


;-----------------------------------------------------------------------
USB_SETUP_XLK_Strobe:	mov	C,Pin_Acknowledge			;(2) Strobe-Signal zum Host senden
			mov	bLastAck,C				;Pegel vom Acknowledge-Bit speichern
			call	Send_Strobe				;Strobe senden
			mov	bLength,#0				;Länge der Antwort setzen
			jmp	USB_Xlink_Break				;und Routine beenden


;-----------------------------------------------------------------------
USB_SETUP_XLK_Ack:	clr	A					;(3) Acknowledge-Bit an den Host senden
			mov	wRequest_LengthLo,A
			mov	wRequest_LengthHi,A

.Ack_Wait:		clr	A
			mov	C,Pin_Acknowledge
			jnb	bLastAck,.Ack_Cont
			cpl	C
.Ack_Cont:		addc	A,#0					;A=A+0+C (A=1, wenn C gesetzt)

.transfer:		mov	DPTR,#EP0_Buffer_Base+0
			movx	@DPTR,A
			jz	+
			cpl	bLastAck				;ggf. aktuelles Ack-Bit als "letzten" Wert speichern
+			mov	UEP0_T_LEN,#1
			mov	bLength,#1				;Länge der Antwort setzen
			mov	UEP0_CTRL,#0C0h
			jmp	USB_Transfer_done


;-----------------------------------------------------------------------
USB_SETUP_XLK_Input:	mov	P1_DIR_PU,#0FFh				;(4) Port P1 als Eingang schalten
			mov	P1_MOD_OC,#0FFh				;Port P1 auf Eingang setzen (mit Pull-Up an den Eingängen)
			mov	P1,#0FFh
			mov	bLength,#0				;Länge der Antwort setzen
			jmp	USB_Xlink_Break


;-----------------------------------------------------------------------
USB_SETUP_XLK_Output:	mov	P1_DIR_PU,#0FFh				;(5) Port P1 als Ausgang schalten
			mov	P1_MOD_OC,#000h				;Port P1 auf Ausgang setzen (Push/Pull-Mode)
			mov	P1,#0FFh
			mov	bLength,#0				;Länge der Antwort setzen
			jmp	USB_Xlink_Break


;-----------------------------------------------------------------------
USB_SETUP_XLK_Read:	mov	DPTR,#EP0_Buffer_Base+0			;(6) Read Byte from Port
			mov	A,P1					;Byte vom Port lesen und im Buffer ablegen
			movx	@DPTR,A
			clr	A
			mov	wRequest_LengthLo,A
			mov	wRequest_LengthHi,A
			inc	A
			mov	bLength,A				;Länge der Antwort setzen
			mov	UEP0_T_LEN,A
			mov	UEP0_CTRL,#0C0h				;#bUEP_R_TOG or bUEP_T_TOG	;0xC0
			jmp	USB_Transfer_done


;-----------------------------------------------------------------------
USB_SETUP_XLK_Write:	mov	DPTR,#EP0_Buffer_Base+2			;(7) Write Byte to Port
			movx	A,@DPTR
			mov	P1,A
			mov	bLength,#0				;Länge der Antwort setzen
			jmp	USB_Xlink_Break


;-----------------------------------------------------------------------
USB_SETUP_XLK_Send:	mov	DPTR,#EP0_Buffer_Base+6			;(8) Send Bytes to Port
			movx	A,@DPTR
			mov	XLink_TransferLength_LO,A		;Gesamtanzahl der zu übertragenen Bytes holen und speichern
			inc	DPTR
			movx	A,@DPTR
			mov	XLink_TransferLength_HI,A
			jmp	USB_Xlink_Break


;-----------------------------------------------------------------------
USB_SETUP_XLK_Receive:	mov	DPTR,#EP0_Buffer_Base+6			;(9) Receive Bytes from Port
			movx	A,@DPTR
			mov	XLink_TransferLength_LO,A		;Gesamtanzahl der zu empfangenen Bytes holen und speichern
			inc	DPTR
			movx	A,@DPTR
			mov	XLink_TransferLength_HI,A

			mov	Length,#EP0_Length			;Länge mit Maximalwerten vorbelegen
			mov	A,XLink_TransferLength_LO		;ist eines der oberen zwei Bits im unteren Byte gesetzt? (dann Wert >64)
			anl	A,#0C0h					
			orl	A,XLink_TransferLength_HI		;ist irgendein Bit im oberen Byte gesetzt? (dann Wert >255)
			jnz	.Continue				;ja, also springen und Maximalwert verwenden
			mov	Length,XLink_TransferLength_LO		;ansonsten ist der Wert <64 und wir dürfen nur die exakte Byte-Anzahl übertragen

.Continue:		mov	Counter,Length				;Länge als Zähler für dieses erste Paket verwenden

			clr	C					;Länge des Pakets von der Gesamtlänge subtrahieren
			mov	A,XLink_TransferLength_LO
			subb	A,Counter
			mov	XLink_TransferLength_LO,A
			mov	A,XLink_TransferLength_HI
			subb	A,#0
			mov	XLink_TransferLength_HI,A

			mov	DPTR,#EP0_Buffer_Base+0			;Basisadresse des Buffers in den DPTR laden

.Loop:			nop
.Ack_Wait:		mov	C,Pin_Acknowledge			;auf Acknowledge warten
			jnb	bLastAck,.Ack_Cont
			cpl	C
.Ack_Cont:		jnc	.Ack_Wait
			cpl	bLastAck				;aktuelles Ack-Bit ist das invertierte alte Ack-Bit

			mov	A,P1					;Byte vom Port lesen, im Buffer ablegen und den Pointer inkrementieren
.Store			movx	@DPTR,A
			inc	DPTR

			call	Send_Strobe				;Strobe senden

			djnz	Counter,.Loop				;restliche Bytes in den Buffer übertragen

			mov	UEP0_T_LEN,Length			;Länge des IN-Pakets setzen
			mov	UEP0_CTRL,#0C0h				;#bUEP_R_TOG or bUEP_T_TOG	;0xC0
			jmp	USB_Transfer_done


;-----------------------------------------------------------------------
USB_Xlink_Break:	clr	A
			mov	wRequest_LengthLo,A
			mov	wRequest_LengthHi,A
			mov	UEP0_T_LEN,A

			mov	UEP0_CTRL,#0C0h				;#bUEP_R_TOG or bUEP_T_TOG	;0xC0
			jmp	USB_Transfer_done


;-----------------------------------------------------------------------
USB_Transfer_is_IN:	mov	A,bUSB_Setup_Cmd
			cjne	A,#015h,+
			jmp	USB_IN_Status_Address
+			cjne	A,#019h,+
			jmp	USB_IN_Status_Config
+			cjne	A,#090h,+
			jmp	USB_IN_Transfer_Data
+			cjne	A,#096h,+
			call	USB_Irq_PrepDescPacket
			xrl	UEP0_CTRL,#040h
			jmp	USB_Transfer_done
			jmp	USB_In_Transfer_Data

+			cjne	A,#051h,+				;XLink Reset
			jmp	USB_IN_Send_ZLP				;nur ein ZLP senden und Transfer beenden
+			cjne	A,#052h,+				;XLink Strobe
			jmp	USB_IN_Send_ZLP				;nur ein ZLP senden und Transfer beenden
+			cjne	A,#054h,+				;XLink Input
			jmp	USB_IN_Send_ZLP				;nur ein ZLP senden und Transfer beenden
+			cjne	A,#055h,+				;XLink Output
			jmp	USB_IN_Send_ZLP				;nur ein ZLP senden und Transfer beenden
+			cjne	A,#057h,+				;XLink Write
			jmp	USB_IN_Send_ZLP				;nur ein ZLP senden und Transfer beenden
+			cjne	A,#058h,+				;XLink Send
			jmp	USB_IN_Send_ZLP				;nur ein ZLP senden und Transfer beenden
+			cjne	A,#0D3h,+				;XLink Acknowledge
			jmp	USB_IN_Send_ZLP				;nur ein ZLP senden und Transfer beenden
+			cjne	A,#0D6h,+				;XLink Read
			jmp	USB_IN_Read_Port
+			cjne	A,#0D9h,+				;XLink Receive
			jmp	USB_IN_XLK_Receive

+			jmp	USB_Irq_Ep0In_default			;Idee für später: hier ein STALL senden, da Kommando nicht bekannt ist...


;-----------------------------------------------------------------------
USB_IN_Read_Port:	nop						;XLink Read (wird aktuell nicht verwendet)

			jmp	USB_IN_Send_ZLP			;DEBUG

			mov	DPTR,#EP0_Buffer_Base+0			;Byte aus Read-Befehl zurückgeben
			mov	A,P1
			movx	@DPTR,A
			mov	UEP0_T_LEN,#1
			xrl	UEP0_CTRL,#040h
			jmp	USB_Transfer_done


;-----------------------------------------------------------------------
USB_IN_XLK_Receive:	mov	A,XLink_TransferLength_HI
			orl	A,XLink_TransferLength_LO		;letztes Paket? Dann ist A=0
			jnz	+
			mov	UEP0_T_LEN,#0
			mov	UEP0_CTRL,#0C0h				;letztes Paket! Nächstes Paket ist dann DATA1
			jmp	USB_Transfer_done

+			mov	Length,#EP0_Length			;Variablen mit Maximalwerten vorbelegen
			mov	A,XLink_TransferLength_LO		;ist eines der oberen zwei Bits im unteren Byte gesetzt? (dann Wert >64)
			anl	A,#0C0h					
			orl	A,XLink_TransferLength_HI		;ist irgendein Bit im oberen Byte gesetzt? (dann Wert >255)
			jnz	.Continue				;ja, also springen und Maximalwert verwenden
			mov	Length,XLink_TransferLength_LO		;ansonsten ist der Wert <64 und wir dürfen nur die exakte Byte-Anzahl übertragen

.Continue:		mov	Counter,Length				;Länge als Zähler für dieses Paket verwenden

			clr	C					;Länge des Pakets von der Gesamtlänge subtrahieren
			mov	A,XLink_TransferLength_LO
			subb	A,Counter
			mov	XLink_TransferLength_LO,A
			mov	A,XLink_TransferLength_HI
			subb	A,#0
			mov	XLink_TransferLength_HI,A

			mov	DPTR,#EP0_Buffer_Base+0			;Basisadresse des Buffers in den DPTR laden

.Loop:			nop
.Ack_Wait:		mov	C,Pin_Acknowledge			;auf Acknowledge warten
			jnb	bLastAck,.Ack_Cont
			cpl	C
.Ack_Cont:		jnc	.Ack_Wait
			cpl	bLastAck				;aktuelles Ack-Bit ist das invertierte alte Ack-Bit

			mov	A,P1					;Byte vom Port lesen, im Buffer ablegen und den Pointer inkrementieren
			movx	@DPTR,A
			inc	DPTR

			call	Send_Strobe				;Strobe senden

			djnz	Counter,.Loop				;restliche Bytes in den Buffer übertragen

			mov	UEP0_T_LEN,Length
			xrl	UEP0_CTRL,#040h				;DATA0/DATA1 togglen
			jmp	USB_Transfer_done


;-----------------------------------------------------------------------
USB_IN_Transfer_Data:	call	USB_Irq_PrepDescPacket
			xrl	UEP0_CTRL,#040h
			jmp	USB_Transfer_done


;-----------------------------------------------------------------------
USB_IN_Send_ZLP:	clr	A					;send a ZLP 
			mov	UEP0_T_LEN,A
			mov	UEP0_CTRL,#UEP_R_RES_ACK|UEP_T_RES_NAK	;folgende OUT-/SETUP-Pakete mit ACK beantworten, folgende IN-Pakete mit NAK beantworten
			jmp	USB_Transfer_done


;-----------------------------------------------------------------------
USB_IN_Status_Address:	mov	A,wValueLo				;neu zugewiesene Adresse speichern
			anl	A,#07Fh
			mov	USB_DEV_AD,A
			mov	UEP0_CTRL,#UEP_R_RES_ACK|UEP_T_RES_NAK	;folgende OUT-/SETUP-Pakete mit ACK beantworten, folgende IN-Pakete mit NAK beantworten
			jmp	USB_Transfer_done


;-----------------------------------------------------------------------
USB_IN_Status_Config:	clr	A					;send a ZLP 
			mov	UEP0_T_LEN,A
			mov	UEP0_CTRL,#UEP_R_RES_ACK|UEP_T_RES_NAK	;folgende OUT-/SETUP-Pakete mit ACK beantworten, folgende IN-Pakete mit NAK beantworten
			jmp	USB_Transfer_done


;-----------------------------------------------------------------------
USB_Irq_Ep0In_default:	clr	A					;send a ZLP 
			mov	UEP0_T_LEN,A
			mov	UEP0_CTRL,#UEP_R_RES_ACK|UEP_T_RES_NAK	;folgende OUT-/SETUP-Pakete mit ACK beantworten, folgende IN-Pakete mit NAK beantworten
			jmp	USB_Transfer_done


;-----------------------------------------------------------------------
USB_Transfer_is_OUT:	mov	A,bUSB_Setup_Cmd
			cjne	A,#058h,+
			jmp	USB_OUT_XLK_Send

+			mov	UEP0_CTRL,#UEP_R_RES_ACK|UEP_T_RES_NAK	;folgende OUT-/SETUP-Pakete mit ACK beantworten, folgende IN-Pakete mit NAK beantworten
			jmp	USB_Transfer_done


;-----------------------------------------------------------------------
USB_OUT_XLK_Send:	mov	Counter,USB_RX_LEN			;Anzahl der Bytes im Out-Paket holen und als Zähler verwenden

			clr	C					;Länge des Pakets von der Gesamtlänge subtrahieren
			mov	A,XLink_TransferLength_LO
			subb	A,Counter
			mov	XLink_TransferLength_LO,A
			mov	A,XLink_TransferLength_HI
			subb	A,#0
			mov	XLink_TransferLength_HI,A

			mov	DPTR,#EP0_Buffer_Base+0			;Basisadresse des Buffers in den DPTR laden

.Loop:			movx	A,@DPTR					;Byte aus Buffer lesen
			inc	DPTR			
			mov	P1,A					;Byte auf Port legen

.Read_Ack_Level:	mov	C,Pin_Acknowledge			;Pegel vom Acknowledge-Bit holen
			mov	bLastAck,C

			call	Send_Strobe				;Strobe senden

.Ack_Wait:		mov	C,Pin_Acknowledge			;auf Acknowledge warten
			jnb	bLastAck,.Ack_Test
			cpl	C
.Ack_Test:		jnc	.Ack_Wait

			djnz	Counter,.Loop				;restliche Bytes aus dem Buffer übertragen

			xrl	UEP0_CTRL,#080h				;DATA0/DATA1 togglen für folgende OUT-Transfers
			orl	UEP0_CTRL,#040h				;inner DATA1 für folgende IN-Transfers
			jmp	USB_Transfer_done


;-----------------------------------------------------------------------
USB_Irq_PrepDescPacket:	clr	C
			mov	A,wRequest_LengthLo			;Paket vorbereiten
			subb	A,#64					;passt das Paket komplett in den Puffer?
			mov	A,wRequest_LengthHi
			subb	A,#0
			jc	USB_Irq_PrepDescPack_1			;ja, dann springen
			mov	R7,#64					;nein, dann Paket auf 64 Bytes (Puffergröße) begrenzen
			jmp	USB_Irq_PrepDescPack_2

USB_Irq_PrepDescPack_1:	mov	R7,wRequest_LengthLo			;exakte Paketlänge nach R7
USB_Irq_PrepDescPack_2:	xch	A,R6
			mov	A,R7					;Paketlänge (0-64) in den Akku kopieren
			xch	A,R6

			mov	my_memcpy_SRC_HI,Descriptor_Address_HI	; src addr
			mov	my_memcpy_SRC_LO,Descriptor_Address_LO
			mov	A,R6					; length

MemCopy:		jz	MemCopy_End				;wenn A=0 (length=0), dann nichts kopieren ==> springe zum Ende der Kopier-Routine
			mov	Counter,A
			mov	XBUS_AUX,#000h				;DPTR0 selektieren
			mov	DPL,my_memcpy_SRC_LO			;DPTR0 auf Source setzen
			mov	DPH,my_memcpy_SRC_HI
			mov	XBUS_AUX,#001h				;DPTR1 selektieren
			mov	DPTR,#EP0_Buffer_Base			;DPTR1 auf Destination setzen
MemCopy_Loop:		mov	XBUS_AUX,#000h				;Datum mit DPTR0 aus code laden
			clr	A
			movc	A,@A+DPTR
			inc	DPTR
			mov	XBUS_AUX,#005h				;Datum mit DPTR1 ins xmem speichern (plus DPTR=DPTR+1)
			movx	@DPTR,A
			djnz	Counter,MemCopy_Loop			;goto loop
MemCopy_End:		mov	XBUS_AUX,#000h

			clr	C					;wRequest_Length = wRequest_Length - R6 (16 Bit Subtraktion)
			mov	A,wRequest_LengthLo
			subb	A,R6
			mov	wRequest_LengthLo,A
			mov	A,wRequest_LengthHi
			subb	A,#0
			mov	wRequest_LengthHi,A

			mov	A,R6					;Zeiger um Anzahl der Bytes (R6) nachführen
			add	A,Descriptor_Address_LO
			mov	Descriptor_Address_LO,A
			clr	A
			addc	A,Descriptor_Address_HI
			mov	Descriptor_Address_HI,A

USB_Irq_PrepDescPack_3:	mov	UEP0_T_LEN,R6				;Anzahl des zu übertragenen Bytes ins entsprechende Register schreiben
			ret



;----------------------------------------------------------------
;
; USB-Diskriptoren
;
;----------------------------------------------------------------

Descriptor_Device_Len	EQU	18					;Geräte-Diskriptor
Descriptor_Device:	DB	Descriptor_Device_Len
			DB	001h
			DW	00110h
			DB	0FFh
			DB	000h
			DB	000h,64
			DW	01D50h
			DW	060C8h
			DW	00100h
			DB	1,2,3
			DB	1


Descriptor_Config:	DB	9,002h
			DB	lo(18),hi(18)
			DB	001h,001h
			DB	000h,0C0h
			DB	50
Descriptor_Interface:	DB	9					;Schnittstellen-Diskriptor folgt direkt auf den Konfigurations-Diskriptor
			DB	004h,000h
			DB	000h,000h
			DB	0FFh,000h
			DB	000h,000h


Descriptor_String0_Len	EQU	4
Descriptor_String0:	DB	Descriptor_String0_Len,3		;Language String Descriptor
			DB	007h,004h				;German


Descriptor_String1_Len	EQU	36
Descriptor_String1:	DB	Descriptor_String1_Len,3		;Manufacturer String Descriptor
			DW	'T','h','o','m','a','s',' ','T','a','h','s','i','n','-','B','e','y'


Descriptor_String2_Len	EQU	32
Descriptor_String2:	DB	Descriptor_String2_Len,3		;Product String Descriptor
			DW	'X','L','i','n','k','-','I','n','t','e','r','f','a','c','e'


Descriptor_String3_Len	EQU	10
Descriptor_String3:	DB	Descriptor_String3_Len,3		;Serial Number String Descriptor
			DW	'0','0','0','0'


Descriptor_Status:	DB	000h,001h				;Get Status Descriptor


Descriptor_Null:	DB	002h,003h				;Nullstring Descriptor

