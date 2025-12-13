    // CONFIG1H
    #pragma config OSC = INTIO67  // Oscillator Selection bits (HS oscillator)
    #pragma config FCMEN = OFF    // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
    #pragma config IESO = ON      // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

    // CONFIG2L
    #pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
    #pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
    #pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

    // CONFIG2H
    #pragma config WDT = OFF  // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
    #pragma config WDTPS = 1  // Watchdog Timer Postscale Select bits (1:1)

    // CONFIG3H
    #pragma config CCP2MX = PORTC  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
    #pragma config PBADEN = ON     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
    #pragma config LPT1OSC = OFF   // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
    #pragma config MCLRE = ON      // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

    // CONFIG4L
    #pragma config STVREN = ON  // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
    #pragma config LVP = OFF    // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
    #pragma config XINST = OFF  // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

    // CONFIG5L
    #pragma config CP0 = OFF  // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
    #pragma config CP1 = OFF  // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
    #pragma config CP2 = OFF  // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
    #pragma config CP3 = OFF  // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

    // CONFIG5H
    #pragma config CPB = OFF  // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
    #pragma config CPD = OFF  // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

    // CONFIG6L
    #pragma config WRT0 = OFF  // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
    #pragma config WRT1 = OFF  // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
    #pragma config WRT2 = OFF  // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
    #pragma config WRT3 = OFF  // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

    // CONFIG6H
    #pragma config WRTC = OFF  // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
    #pragma config WRTB = OFF  // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
    #pragma config WRTD = OFF  // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

    // CONFIG7L
    #pragma config EBTR0 = OFF  // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
    #pragma config EBTR1 = OFF  // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
    #pragma config EBTR2 = OFF  // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
    #pragma config EBTR3 = OFF  // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

    // CONFIG7H
    #pragma config EBTRB = OFF  // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

    #include <ctype.h>
    #include <pic18f4520.h>
    #include <stdbool.h>
    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
    #include <xc.h>
    #define _XTAL_FREQ 8000000
    #define STR_MAX 100
    #define VR_MAX ((1 << 10) - 1)
    // #define delay(t) __delay_ms(t * 1000);
    #define MAX30102_WRITE  (0xAE) // 0x57 << 1
    #define MAX30102_READ   (0xAF)

    //---------------- Global Variebles --------------------

    char buffer[STR_MAX];
    int buffer_size = 0;
    bool btn_interr = false;



    //---------------- Functions --------------------

    // ===== UART functions =====
    void putch(char data);
    void ClearBuffer();
    void MyusartRead();
    int GetString(char *str);
    void clear_terminal();

    // ===== Callback functions =====
    void variable_register_changed(int value);
    void button_pressed();
    void keyboard_input(char *str);

    // ===== Utility =====
    int delay(double sec);

    // ---------------- Settings --------------------

    void ADC_Initialize(void){
        // Configure ADC
        TRISAbits.RA0 = 1;         // Set RA0 as input port
        ADCON0bits.CHS = 0b0000;   // Select AN0 channel
        ADCON1bits.VCFG0 = 0;      // Vref+ = Vdd
        ADCON1bits.VCFG1 = 0;      // Vref- = Vss
        ADCON2bits.ADCS = 0b000;   // ADC clock Fosc/2 Tad = 2us
        ADCON2bits.ACQT = 0b001;   // 2Tad acquisition time 4us
        ADCON0bits.ADON = 1;       // Enable ADC
        ADCON2bits.ADFM = 1;       // Right justified
        PIR1bits.ADIF = 0;         // Clear ADC flag
        IPR1bits.ADIP = 0;         // Set ADC interrupt priority to low
        PIE1bits.ADIE = 1;         // Enable ADC interrupt
    }


    void Button_Initialize(void){
        INTCONbits.INT0IF = 0;  // Clear INT0 flag
        INTCONbits.INT0IE = 1;  // Enable INT0 interrupt
        //INTCON2bits.INTEDG0 = 1; // rising edge triggered (pressed)
        TRISB |= 0x01;      // RB0 as input for button
    }


    void UART_Initialize(void){
        // Configure UART
        /*
            TODObasic
            Serial Setting
            1.   Setting Baud rate
            2.   choose sync/async mode
            3.   enable Serial port (configures RX/DT and TX/CK pins as serial port pins)
            3.5  enable Tx, Rx Interrupt(optional)
            4.   Enable Tx & RX
        */
        TRISCbits.TRISC6 = 1;  // RC6(TX) : Transmiter set 1 (output)
        TRISCbits.TRISC7 = 1;  // RC7(RX) : Receiver set 1   (input)

        // Setting Baud rate (when fosc = 8M)
        // Baud rate = 9600 (Look up table)
        TXSTAbits.SYNC = 0;     // Synchronus or Asynchronus
        BAUDCONbits.BRG16 = 0;  // 16 bits or 8 bits
        TXSTAbits.BRGH = 1;     // High Baud Rate Select bit
        SPBRG = 51;             // Control the period
        //SPBRGH = ;            // Used when BRG16 = 1 

        // Serial enable
        RCSTAbits.SPEN = 1;  // Enable asynchronus serial port (must be set to 1)
        PIR1bits.TXIF = 0;   // Set when TXREG is empty
        PIR1bits.RCIF = 0;   // Will set when reception is complete
        TXSTAbits.TXEN = 1;  // Enable transmission
        RCSTAbits.CREN = 1;  // Continuous receive enable bit, will be cleared when error occured
        PIE1bits.TXIE = 0;   // Wanna use Interrupt (Transmit)
        IPR1bits.TXIP = 0;   // Interrupt Priority bit
        PIE1bits.RCIE = 1;   // Wanna use Interrupt (Receive)
        IPR1bits.RCIP = 1;   // Interrupt Priority bit
        /* Transmitter (output)
        TSR   : Current Data
        TXREG : Next Data
        TXSTAbits.TRMT will set when TSR is empty
        */
        /* Reiceiver (input)
        RSR   : Current Data
        RCREG : Correct Data (have been processed) : read data by reading the RCREG Register
        */

    }

    void I2C_Initialize(void) {
        TRISCbits.RC3 = 1;      // serial port: SCL
        TRISCbits.RC4 = 1;      // serial port: SDA
        SSPSTATbits.SMP = 1;    // standard speed: 100 kHz or 1MHz
        SSPSTATbits.CKE = 1;    // disable SMBus
        SSPCON1bits.SSPM = 8;   // master mode
        SSPADD = 19;            // @todo adjust Fosc and this speed: 100khz
        SSPCON1bits.SSPEN = 1;  // enable serial ports: SCL, SDA
    }

    void Initialize(void) {
        OSCCONbits.IRCF = 0b111;  // Osc = 4 MHz
        ADCON1bits.PCFG = 0b1110;  // AN0 as analog input

        //ADC_Initialize();
        //Button_Initialize();
        UART_Initialize();
        I2C_Initialize();

        // Configure interrupts
        INTCONbits.PEIE = 1;    // Enable peripheral interrupt
        RCONbits.IPEN = 1;      // enable Interrupt Priority mode
        INTCONbits.GIEH = 1;    // enable high priority interrupt
        INTCONbits.GIEL = 1;    // enable low priority interrupt
        INTCONbits.GIE = 1;     //Enable Global Interrupt
    }

    // ---------------- OOP --------------------

    // ---------------- Uart --------------------

    void putch(char data) {  // Output on Terminal
        if (data == '\n' || data == '\r') {
            while (!TXSTAbits.TRMT);
            TXREG = '\r';
            while (!TXSTAbits.TRMT);
            TXREG = '\n';
        } else {
            while (!TXSTAbits.TRMT);
            TXREG = data;
        }
    }

    void ClearBuffer() {
        for (int i = 0; i < STR_MAX; i++)
            buffer[i] = '\0';
        buffer_size = 0;
    }

    void MyusartRead() {
        char data = RCREG;
        if (!isprint(data) && data != '\r') return;
        buffer[buffer_size++] = data;
        //putch(data);
    }

    int GetString(char *str) {
        if (buffer[buffer_size - 1] == '\r') {
            buffer[--buffer_size] = '\0';
            strcpy(str, buffer);
            ClearBuffer();
            return 1;
        } else {
            str[0] = '\0';
            return 0;
        }
    }

    void clear_terminal() {
        printf("\033[2J\033[H");  
    }
    // ---------------- I2C --------------------

void i2c_is_idle(void)
{
    while ((SSPCON2 & 0x1F) || (SSPSTATbits.R_W));
}

void i2c_start(void){
    i2c_is_idle();
    SSPCON2bits.SEN = 1;
}

void i2c_rep_start(void){
    i2c_is_idle();
    SSPCON2bits.RSEN = 1;
}

void i2c_stop(void){
    i2c_is_idle();
    SSPCON2bits.PEN = 1;
}

void i2c_write(uint8_t i2c_data){
    i2c_is_idle();
    SSPBUF = i2c_data;
    while(SSPSTATbits.BF != 0);
    while(SSPCON2bits.ACKSTAT != 0);
}

uint8_t i2c_read(uint8_t ack){
    uint8_t recieve =0;
    i2c_is_idle();
    SSPCON2bits.RCEN = 1;
    while(SSPSTATbits.BF != 1);
    recieve = SSPBUF;
    SSPCON2bits.ACKEN = ack;
    return recieve;
} 
 // ---------------- Delay --------------------

    int delay(double sec) {
        btn_interr = false;
        int loops = (int)(sec * 1000 / 10);
        for (int i = 0; i < loops; i++) {
            if (btn_interr) return -1;
            __delay_ms(10);
        }
        return 0;
    }
    // ---------------- Interrupt --------------------
    void __interrupt(high_priority) H_ISR(void) {

        if (INTCONbits.INT0IF) {  // Handle button interrupt
            INTCONbits.INT0IE = 0;  
            button_pressed();
            __delay_ms(50);  // bouncing problem
            btn_interr = true;
            INTCONbits.INT0IF = 0;
            INTCONbits.INT0IE = 1;  
        }

        if (RCIF) {
            if (RCSTAbits.OERR) {
                CREN = 0;
                Nop();
                CREN = 1;
            }

            MyusartRead();
        }
    }

    void __interrupt(low_priority) Lo_ISR(void) {
        if (PIR1bits.ADIF) {  // Handle variable register interrupt
            int value = (ADRESH << 8) + ADRESL;
            variable_register_changed(value);
            PIR1bits.ADIF = 0;
            __delay_ms(5);    // Larger then 2tad
        }

        // process other interrupt sources here, if required
        return;
    }

    // --------------- TODO ------------------

    void button_pressed(void) {
        // Do sth when the button is pressed
        /* Example:
        * set_LED(get_LED() + 1);
        */


    }

    void variable_register_changed(int value) {  // value: 0 ~ 1023
        // Do sth when the variable register changes
        /* Example:
        * set_servo_angle(VR_value_to_servo_angle(value));
        * set_LED_analog(VR_value_to_LED_analog(value));
        * printf("%d\n", value); // print the variable register value on uart terminal
        */


    }

    void keyboard_input(char *str) {  // get line from keyboard: this function will be called after you click enter
        // Do sth when typing on keyboard
        /* Example:
            1.
            if(strcmp(str, "mode1") == 0) {
                mode = 1;
            } else if(strcmp(str, "mode2") == 0) {
                mode = 2;
            }
            2.
            sscanf(str, "%d %d", &parsed_state, &parsed_count)
        */
        


    }

    void timer0_interrupted(void) {  // Do sth when timer0 is interrupted
        
    }

    void timer1_interrupted(void) {  // Do sth when timer1 is interrupted
        
    }

    void timer2_interrupted(void) {  // Do sth when timer2 is interrupted
        
    }

    void timer3_interrupted(void) {  // Do sth when timer3 is interrupted
        
    }

    void main() {
        Initialize();
        /* Usage:
        * delay(1); // delay 1 second, return value: -1 represents interrupt with button press, else 0
        *
        * printf(); // print on uart terminal
        */

        uint8_t sample_data[6]; // 存放 6 bytes (3 bytes RED, 3 bytes IR)
        uint32_t red_val = 0;
        uint32_t ir_val = 0;

        char str[STR_MAX];
        
        // Mode Configuration (0x09), RESET=1
        // 1. Reset MAX30102
        /*i2c_start();
        if (!i2c_write(MAX30102_WRITE)) { printf("Err: Write Addr\n"); }
        if (!i2c_write(0x09))           { printf("Err: Reg 0x09\n"); }
        if (!i2c_write(0x40))           { printf("Err: Data 0x40\n"); }
        i2c_stop();*/

        printf("Check Sensor ID...\n");
    
        uint8_t part_id;
        
        // 步驟 1: 設定指標到 0xFF
        i2c_start();
        printf("yeah\n");
        i2c_write(MAX30102_WRITE);
        printf("yeah\n");
        i2c_write(0xFF); // Part ID Register
        printf("yeah\n");
        // 步驟 2: 讀取資料
        i2c_rep_start();
        printf("yeah\n");
        i2c_write(MAX30102_READ);
        printf("yeah\n");
        part_id = i2c_read(0); // 0 = NACK (只讀 1 byte 就停)
        printf("yeah\n");
        i2c_stop();
        
        printf("Part ID: 0x%02X\n", part_id);

        if(part_id == 0x15) {
            printf("MAX30102 is ONLINE! \n");
        } else {
            printf("Wrong ID or Bus Error.\n");
        }
        
        __delay_ms(1000); // 停一下讓你看結果

        // Mode Configuration (0x09), RESET=1
        i2c_start();
        i2c_write(MAX30102_WRITE);
        i2c_write(0x09);
        i2c_write(0x40);  // RESET bit
        i2c_stop();
        __delay_ms(10);

        i2c_start();
        i2c_write(MAX30102_WRITE);
        i2c_write(0x04); // FIFO_WR_PTR
        i2c_write(0x00);
        i2c_write(0x00); // OVF_COUNTER
        i2c_write(0x00); // FIFO_RD_PTR
        i2c_stop();

        // FIFO_CONFIG = 0x00
        i2c_start();
        i2c_write(MAX30102_WRITE);
        i2c_write(0x08);
        i2c_write(0x00);
        i2c_stop();

        // SpO2 Config
        // ADC range = 4096nA (01)
        // Sample rate = 100 SPS (001)
        // LED pulse width = 411us (11, 18-bit)
        i2c_start();
        i2c_write(MAX30102_WRITE);
        i2c_write(0x0A);
        i2c_write(0b01000111);
        i2c_stop();

        // Red LED
        i2c_start();
        i2c_write(MAX30102_WRITE);
        i2c_write(0x0C);
        i2c_write(0x24); // 約 7mA
        i2c_stop();

        // IR LED
        i2c_start();
        i2c_write(MAX30102_WRITE);
        i2c_write(0x0D);
        i2c_write(0x24);
        i2c_stop();

        // MODE = 0b011 (SpO2 mode)
        i2c_start();
        i2c_write(MAX30102_WRITE);
        i2c_write(0x09);
        i2c_write(0x03);
        i2c_stop();



        while (1) {
        // Do sth in main
        /*// If typing is detected, pause system operations until Enter is pressed
        if (buffer_size > 0) {
            while (!GetString(str)) {}
            keyboard_input(str);
        }*/
        //if (GetString(str)) keyboard_input(str);
        //if (ADCON0bits.GO == 0) ADCON0bits.GO = 1;
        uint8_t wr, rd;
        uint8_t samples;

        // 1️⃣ 讀 FIFO WR_PTR
        i2c_start();
        i2c_write(MAX30102_WRITE);
        i2c_write(0x04);
        i2c_rep_start();
        i2c_write(MAX30102_READ);
        wr = i2c_read(0);
        i2c_stop();

        // 2️⃣ 讀 FIFO RD_PTR
        i2c_start();
        i2c_write(MAX30102_WRITE);
        i2c_write(0x06);
        i2c_rep_start();
        i2c_write(MAX30102_READ);
        rd = i2c_read(0);
        i2c_stop();

        samples = (wr - rd) & 0x1F;

        // 3️⃣ 有資料才讀
        if(samples > 0)
        {
            uint32_t red, ir;

            i2c_start();
            i2c_write(MAX30102_WRITE);
            i2c_write(0x07);
            i2c_rep_start();
            i2c_write(MAX30102_READ);

            red  = ((uint32_t)i2c_read(1)) << 16;
            red |= ((uint32_t)i2c_read(1)) << 8;
            red |= i2c_read(1);

            ir   = ((uint32_t)i2c_read(1)) << 16;
            ir  |= ((uint32_t)i2c_read(1)) << 8;
            ir  |= i2c_read(0);

            i2c_stop();

            red &= 0x3FFFF;
            ir  &= 0x3FFFF;

            printf("RED=%lu IR=%lu\n", red, ir);


        __delay_ms(10); // 👈 非常重要
            
        }

    }
}
    