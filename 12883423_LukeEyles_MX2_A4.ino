#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdlib.h>
#include <LiquidCrystal.h>

#define bit_set(reg, rbit) (reg |= 1 << rbit)
#define bit_clear(reg, rbit) (reg &= ~(1 << rbit))
#define bit_toggle(reg, rbit) (reg ^= (1 << rbit))
#define bit_check(reg, rbit) ((reg >> rbit) & 1)

#define line_blank "                "

// modes
#define MODE_START 0
#define MODE_INVALID 4
#define MODE_USER_MENU 1
#define MODE_USER_CLOSEST 11
#define MODE_USER_LOCK_CLOSEST 12
#define MODE_USER_MAP 15
#define MODE_USER_SEC 16
#define MODE_USER_INTRUDER 17
#define MODE_ADMIN_MENU 2
#define MODE_ADMIN_ORIGIN 21
#define MODE_ADMIN_MAP_MENU 25
#define MODE_ADMIN_MAP 22
#define MODE_ADMIN_SEC 23
#define MODE_ADMIN_TIMED 24
#define MODE_ADMIN_TIMED_MENU 27
#define MODE_ADMIN_INTRUDER 26
#define MODE_DEV_MENU 3
#define MODE_DEV_STEPPER 31
#define MODE_DEV_FIND 32
#define MODE_DEV_LOCK_FIND 34
#define MODE_DEV_FIND_MENU 35
#define MODE_DEV_TRACK_MENU 36
#define MODE_DEV_TRACK 33
#define MODE_DEV_TRACK_DISPLAY 37
#define MODE_LOCKOUT 5
#define MODE_PC 6
#define MODE_PC_PASSCODE 61
#define MODE_PC_MAP 62
#define MODE_PC_CHANGE 63
#define MODE_PC_LOCAL 64
#define MODE_PC_ORIGIN 65

// commands (so they are stored in flash)
const char CMD_UTIL_Clock[] PROGMEM = { "CMD_UTIL_Clock" };
const char CMD_UTIL_SID[] PROGMEM = { "CMD_UTIL_SID" };
const char CMD_UTIL_BPW[] PROGMEM = { "CMD_UTIL_BPW" };
const char CMD_UTIL_APW[] PROGMEM = { "CMD_UTIL_APW" };
const char CMD_UTIL_DPW[] PROGMEM = { "CMD_UTIL_DPW" };
const char CMD_UTIL_BPW_RESET[] PROGMEM = { "CMD_UTIL_BPW_RESET" };
const char CMD_UTIL_APW_RESET[] PROGMEM = { "CMD_UTIL_APW_RESET" };
const char CMD_UTIL_DPW_RESET[] PROGMEM = { "CMD_UTIL_DPW_RESET" };
const char CMD_CANCEL[] PROGMEM = { "CMD_CANCEL" };
const char CMD_EXIT[] PROGMEM = { "CMD_EXIT" };
const char CMD_BASIC_DCO[] PROGMEM = { "CMD_BASIC_DCO" };
const char CMD_BASIC_SEC[] PROGMEM = { "CMD_BASIC_SEC" };
const char CMD_BASIC_SM[] PROGMEM = { "CMD_BASIC_SM" };
const char CMD_ADMIN_SOC[] PROGMEM = { "CMD_ADMIN_SOC" };
const char CMD_ADMIN_ASEC[] PROGMEM = { "CMD_ADMIN_ASEC" };
const char CMD_ADMIN_SM[] PROGMEM = { "CMD_ADMIN_SM" };
const char CMD_ADMIN_TSM[] PROGMEM = { "CMD_ADMIN_TSM" };
const char CMD_PC_MAP[] PROGMEM = { "CMD_PC_MAP" };
const char CMD_PC_CHANGE[] PROGMEM = { "CMD_PC_CHANGE" };
const char CMD_PC_LOCAL[] PROGMEM = { "CMD_PC_LOCAL" };
const char CMD_SUBMIT[] PROGMEM = { "CMD_SUBMIT" };

// stepper
#define IN1 PORTD2 // pin 2
#define IN2 PORTD3 // 3
#define IN3 PORTB3 // 11
#define IN4 PORTB4 // 12
#define CW 0
#define CCW 1
#define STEPS_PER_REV 4096.0 // assuming half step mode

// lcd
#define LCD_EN 9
#define LCD_RS 8
#define LCD_7 7
#define LCD_6 6
#define LCD_5 5
#define LCD_4 4
#define LCD_BL 10
#define LCD_LEN 17

// distance sensor
#define NUMAVG 8
#define READINGS_PER_REV 32
#define MAP_SIZE 21
#define CM_PER_CELL 10

// serial
#define readSerialString(buff, n) Serial.readStringUntil('\n').toCharArray(buff,n) // reads serial into c string
#define isCommand(cmdin, cmd) (strcmp_P(cmdin, cmd) == 0)

// setup
void setupIO();
void setupADC();
void setupLCD();
void setupTimer1();
void setupTimer2();

// program specific
void recognisePassword(char password[]);
void printLCD(int arg1, int arg2);
void modeswitch(int nextmode);
void mapArea(int times);
void takeScans(uint8_t output[], int scans);
void goToStep(int goal_step);
uint8_t readDistanceSensor();
long generatePasscode();
bool validatePassword(const char *pw);
void createMap(uint8_t distance[]);
void mapChanges(uint8_t newdist[], uint8_t olddist[]);
void localiseSensor(uint8_t newdist[], uint8_t olddist[]);

// math
long avgArray(int a[], int sz);
int medianArray(int a[], int sz);
uint8_t minArray(uint8_t a[], int sz);
float adc2v(int adc);
float adc2distance(long a);
int myAbs(int n);
long myPow(int base, int power);
int myRound(float num);
float mySqrt(int b);
void myDelay(long time_ms);

volatile long counter = 0;
volatile bool countdown_enabled = false;
volatile int countdown_ms = 0;

volatile int adcarr[NUMAVG]; // array of adc values to be averaged
int iarr = 0; // index in adc array
uint8_t current_distance; // current distance reading
uint8_t dist[READINGS_PER_REV] = {0};
uint8_t mapped_dist[READINGS_PER_REV] = {0}; // for cmd pc change

volatile char button = 'O';
volatile char button_pressed = 'O';
int prev_mode = MODE_START;
int mode = MODE_START;

volatile int current_step = 0;
volatile int start_step = 0;
volatile int stepper_counter = 0;
volatile int stepperspeed = 1000;
volatile int stepperdir = CW;
int origin = 0;
int revolutions = 1;

char pw[] = "________________";
int lcdidx = 0;
int attempts = 0;
int menu_selected = 0;

bool origin_set = false;
bool mapping = false;
int intruder = 0; // location of intruder

char command[20];
bool pc_mode = false; // when in pc mode, input and output through serial
char oldline1[LCD_LEN] = line_blank;
char oldline2[LCD_LEN] = line_blank;

struct Passwords
{
    char userpw[17];
    char adminpw[17];
    char devpw[17];
    long pcpw;
} passwords = {"18007383773", "8055___101", "6006137771011019", 0};

struct invalidModeData
{
    long timetoreset;
};

struct devTrackModeData
{
    uint8_t start_distance;
    uint8_t prev_distance;
    long start_time;
    long delta_time;
};

struct devFindModeData
{
    uint8_t find_distance;
};

struct adminTimedModeData
{
    long timer;
    long reset_time;
};

struct pcModeData
{
    long timer;
};

union modeData // for variables only associated with one mode
{
    invalidModeData invalid_mode;
    devTrackModeData dev_track_mode;
    devFindModeData dev_find_mode;
    adminTimedModeData admin_timed_mode;
    pcModeData pc_mode_data;
};
modeData data;

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_4, LCD_5, LCD_6, LCD_7);

void setup()
{
    cli();

    setupIO();
    setupADC();
    setupTimer1();
    setupLCD();
    setupTimer2();

    sei(); // enable interrupts globally
    Serial.begin(9600);
}

void loop()
{
    if (mode == MODE_START)
    {
        printLCD(0, 0);
        if (button_pressed == 'S')
        {
            // test password
            recognisePassword(pw);
        }
        else if (button_pressed == 'R')
        {
            lcdidx += 1;
            if (lcdidx > 16)
            {
                lcdidx = 16;
            }
        }
        else if (button_pressed == 'L')
        {
            lcdidx -= 1;
            if (lcdidx < 0)
            {
                lcdidx = 0;
            }
        }
        else if (button_pressed == 'U')
        {
            if (pw[lcdidx] == '_')
            {
                pw[lcdidx] = '0';
            }
            else if (pw[lcdidx] != '9')
            {
                // don't increment past 9
                pw[lcdidx] += 1;
            }
        }
        else if (button_pressed == 'D')
        {
            if (pw[lcdidx] == '_')
            {
                pw[lcdidx] = '0';
            }
            else if (pw[lcdidx] != '0')
            {
                // don't decrement past 0
                pw[lcdidx] -= 1;
            }
        }
    }
    else if (mode == MODE_INVALID)
    {
        printLCD(0, 0);
        if (counter >= data.invalid_mode.timetoreset)
        {
            modeswitch(MODE_START);
        }
    }
    else if (mode == MODE_LOCKOUT)
    {
        printLCD(0, 0);
    }
    else if (mode == MODE_USER_MENU)
    {
        printLCD(0, 0);
        int menu[] = {MODE_USER_CLOSEST, MODE_USER_MAP, MODE_USER_SEC, MODE_START};
        if (button_pressed == 'S')
        {
            modeswitch(menu[menu_selected]);
        }
        else if (button_pressed == 'R')
        {
            menu_selected += 1;
            if (menu_selected > 3)
            {
                menu_selected = 3;
            }
        }
        else if (button_pressed == 'L')
        {
            menu_selected -= 1;
            if (menu_selected < 0)
            {
                menu_selected = 0;
            }
        }
    }
    else if (mode == MODE_USER_CLOSEST)
    {
        printLCD(0, 0);
        start_step = current_step;
        mapArea(1);
        // interrupt will turn stepper the correct number of steps then disable stepper

        modeswitch(MODE_USER_LOCK_CLOSEST); // exit this mode once the reading has been taken
    }
    else if (mode == MODE_USER_LOCK_CLOSEST)
    {
        printLCD(0, 0);
        if (button_pressed == 'S')
        {
            memset(dist, 0, sizeof(dist));
            modeswitch(MODE_USER_MENU);
        }
    }
    else if (mode == MODE_USER_MAP)
    {
        mapArea(1);
        if (pc_mode) modeswitch(MODE_PC);
        else modeswitch(MODE_USER_MENU);
    }
    else if (mode == MODE_USER_SEC || mode == MODE_ADMIN_SEC || mode == MODE_ADMIN_TIMED)
    {
        printLCD(0, 0);
        if (mode == MODE_ADMIN_TIMED && counter >= data.admin_timed_mode.reset_time)
        {
            if (pc_mode) modeswitch(MODE_PC);
            else modeswitch(MODE_ADMIN_MENU);
        }
        else if (!origin_set && (mode == MODE_ADMIN_SEC || mode == MODE_ADMIN_TIMED))
        {
            if (pc_mode) modeswitch(MODE_PC_ORIGIN);
            else modeswitch(MODE_ADMIN_ORIGIN);
        }
        else
        {
            // map area if needed
            if (dist[0] == 0) // mapping hasn't been done yet
            {
                mapping = true;
                mapArea(1);
                mapping = false;
            }

            // scan 1 segment
            uint8_t d = readDistanceSensor();

            int loc = READINGS_PER_REV * ((current_step - origin) / STEPS_PER_REV);

            if (loc == (READINGS_PER_REV - 1)) stepperdir = !stepperdir; // reverse direction so cord doesn't tangle

            // compare with map
            if ((dist[loc] - d) > 10)
            {
                intruder = loc;
                if (mode == MODE_USER_SEC) modeswitch(MODE_USER_INTRUDER);
                else modeswitch(MODE_ADMIN_INTRUDER);
            }
            else // rotate motor
            {
                stepper_counter = STEPS_PER_REV / READINGS_PER_REV; // number of steps in 1 interval
                OCR2A = 15625 / stepperspeed;
                bit_set(TIMSK2, OCIE2A); // enable stepper
                while (bit_check(TIMSK2, OCIE2A)); // wait for motor to finish rotating
            }

            if (button_pressed == 'S')
            {
                if (mode == MODE_USER_SEC) modeswitch(MODE_USER_MENU);
                else modeswitch(MODE_ADMIN_MENU);
            }
        }
    }
    else if (mode == MODE_USER_INTRUDER)
    {
        printLCD(0, 0);
        // read distance at detected position of intruder
        dist[intruder] = readDistanceSensor();

        if (button_pressed == 'S')
        {
            goToStep(origin);
            modeswitch(MODE_USER_MENU);
        }
    }
    else if (mode == MODE_ADMIN_MENU)
    {
        printLCD(0, 0);
        int menu[] = {MODE_ADMIN_ORIGIN, MODE_ADMIN_MAP_MENU, MODE_ADMIN_SEC, MODE_ADMIN_TIMED_MENU, MODE_START};
        if (button_pressed == 'S')
        {
            modeswitch(menu[menu_selected]);
        }
        else if (button_pressed == 'R')
        {
            menu_selected += 1;
            if (menu_selected > 4)
            {
                menu_selected = 4;
            }
        }
        else if (button_pressed == 'L')
        {
            menu_selected -= 1;
            if (menu_selected < 0)
            {
                menu_selected = 0;
            }
        }
    }
    else if (mode == MODE_ADMIN_ORIGIN)
    {
        printLCD(0, 0);
        // use left and right buttons to rotate the motor, store the origin step so it can be returned to
        if (button == 'L')
        {
            stepperdir = CCW;
            bit_set(TIMSK2, OCIE2A); // enable stepper
        }
        else if (button == 'R')
        {
            stepperdir = CW;
            bit_set(TIMSK2, OCIE2A); // enable stepper
        }
        else if (button == 'O')
        {
            bit_clear(TIMSK2, OCIE2A); // disable stepper
        }

        if (button_pressed == 'S')
        {
            origin = current_step;
            origin_set = true;
            if (prev_mode == MODE_ADMIN_SEC ||  prev_mode == MODE_ADMIN_TIMED)
            {
                modeswitch(prev_mode);
            }
            else modeswitch(MODE_ADMIN_MENU);
        }
    }
    else if (mode == MODE_ADMIN_MAP_MENU)
    {
        if (button_pressed == 'U' && revolutions < 9) revolutions++;
        else if (button_pressed == 'D' && revolutions > 1) revolutions--;
        else if (button_pressed == 'S')
        {
            modeswitch(MODE_ADMIN_MAP);
        }

        if (pc_mode)
        {
            // check if command is a number between 1 and 9
            if (atoi(command) > 0 && atoi(command) < 10)
            {
                revolutions = atoi(command);
                Serial.print(F("Calibrating with "));
                Serial.print(revolutions);
                Serial.println(F(" revolution/s"));
                modeswitch(MODE_ADMIN_MAP);
            }
        }
        printLCD(0, 0);
    }
    else if (mode == MODE_ADMIN_MAP)
    {
        mapArea(revolutions);
        if (pc_mode) modeswitch(MODE_PC);
        else modeswitch(MODE_ADMIN_MENU);
    }
    else if (mode == MODE_ADMIN_INTRUDER)
    {
        printLCD(intruder, 0);

        // read distance at detected position of intruder
        dist[intruder] = readDistanceSensor();

        if (button_pressed == 'S')
        {
            modeswitch(MODE_ADMIN_MENU);
        }
    }
    else if (mode == MODE_ADMIN_TIMED_MENU)
    {
        printLCD(0, 0);
        if (button_pressed == 'L' && lcdidx > 0) lcdidx--;
        else if (button_pressed == 'R' && lcdidx < 3) lcdidx++;
        else if (button_pressed == 'U')
        {
            if (lcdidx == 0) data.admin_timed_mode.timer += 60 * 10;
            else if (lcdidx == 1) data.admin_timed_mode.timer += 60;
            else if (lcdidx == 2) data.admin_timed_mode.timer += 10;
            else if (lcdidx == 3) data.admin_timed_mode.timer += 1;
        }
        else if (button_pressed == 'D')
        {
            if (lcdidx == 0) data.admin_timed_mode.timer -= 60 * 10;
            else if (lcdidx == 1) data.admin_timed_mode.timer -= 60;
            else if (lcdidx == 2) data.admin_timed_mode.timer -= 10;
            else if (lcdidx == 3) data.admin_timed_mode.timer -= 1;
        }
        else if (button_pressed == 'S')
        {
            data.admin_timed_mode.reset_time = counter + data.admin_timed_mode.timer * 1000;
            Serial.println(data.admin_timed_mode.reset_time);
            modeswitch(MODE_ADMIN_TIMED);
        }

        if (pc_mode)
        {
            // check if command is in format m:ss or mm:ss
            int mins = 0;
            int secs = 0;
            char * ptr1;
            char * ptr2;
            mins = strtol(command, &ptr1, 10);

            if (ptr1 > command)
            {
                if (*ptr1 == ':')
                {
                    secs = strtol(ptr1 + 1, &ptr2, 10);

                    if ( ptr2 > ptr1 + 1)
                    {
                        if (mins < 99 && secs < 59) // valid time
                        {
                            strcpy(command, "");
                            data.admin_timed_mode.timer = mins * 60 + secs;
                            data.admin_timed_mode.reset_time = counter + data.admin_timed_mode.timer * 1000;
                            modeswitch(MODE_ADMIN_TIMED);
                        }

                    }
                }
            }

        }

    }
    else if (mode == MODE_DEV_MENU)
    {
        printLCD(0, 0);
        int menu[] = {MODE_PC_PASSCODE, MODE_DEV_STEPPER, MODE_DEV_FIND_MENU, MODE_DEV_TRACK_MENU, MODE_START};
        if (button_pressed == 'S')
        {
            modeswitch(menu[menu_selected]);
        }
        else if (button_pressed == 'R')
        {
            menu_selected += 1;
            if (menu_selected > 4)
            {
                menu_selected = 4;
            }
        }
        else if (button_pressed == 'L')
        {
            menu_selected -= 1;
            if (menu_selected < 0)
            {
                menu_selected = 0;
            }
        }
    }
    else if (mode == MODE_DEV_STEPPER)
    {
        if (button == 'L')
        {
            stepperdir = CCW;
            bit_set(TIMSK2, OCIE2A); // enable stepper
        }
        else if (button == 'R')
        {
            stepperdir = CW;
            bit_set(TIMSK2, OCIE2A); // enable stepper
        }
        else if (button == 'O')
        {
            bit_clear(TIMSK2, OCIE2A); // disable stepper
        }

        if (button_pressed == 'S')
        {
            modeswitch(MODE_DEV_MENU);
        }
        printLCD(current_distance, 0);
    }
    else if (mode == MODE_DEV_FIND_MENU)
    {
        printLCD(data.dev_find_mode.find_distance, 0);
        // use left and right buttons to adjust distance to search
        if (button_pressed == 'L' && data.dev_find_mode.find_distance > 30) data.dev_find_mode.find_distance -= 5;
        else if (button_pressed == 'R' && data.dev_find_mode.find_distance < 150) data.dev_find_mode.find_distance += 5;
        else if (button_pressed == 'S')
        {
            modeswitch(MODE_DEV_FIND);
        }
    }
    else if (mode == MODE_DEV_FIND)
    {
        printLCD(0, 0);
        start_step = current_step;
        mapArea(1);
        modeswitch(MODE_DEV_LOCK_FIND);

        if (button_pressed == 'S')
        {
            modeswitch(MODE_DEV_MENU);
        }
    }
    else if (mode == MODE_DEV_LOCK_FIND)
    {
        while (bit_check(TIMSK2, OCIE2A)); // wait for motor to finish rotating

        // continue reading distance, if it changes too much then switch back to find mode
        uint8_t d = readDistanceSensor();
        printLCD(d, 0);
        if (myAbs(d - data.dev_find_mode.find_distance) > 5)
        {
            modeswitch(MODE_DEV_FIND);
        }

        if (button_pressed == 'S')
        {
            modeswitch(MODE_DEV_MENU);
        }
    }
    else if (mode == MODE_DEV_TRACK_MENU)
    {
        printLCD(0, 0);
        // wait for select to be pressed
        if (button_pressed == 'S')
        {
            //            timer = counter; // start time
            //            start_distance = readDistanceSensor();
            //            prev_distance = start_distance;
            data.dev_track_mode.start_time = counter;
            data.dev_track_mode.start_distance = readDistanceSensor();
            data.dev_track_mode.prev_distance = data.dev_track_mode.start_distance;

            modeswitch(MODE_DEV_TRACK);
        }
    }
    else if (mode == MODE_DEV_TRACK)
    {
        printLCD(0, 0);
        uint8_t d = readDistanceSensor();

        myDelay(100); // don't read too fast

        if (myAbs(d - data.dev_track_mode.prev_distance) < 2)
        {
            // object has stopped moving
            //timer = counter - timer; // change in time
            data.dev_track_mode.delta_time = counter - data.dev_track_mode.start_time;
            modeswitch(MODE_DEV_TRACK_DISPLAY);
        }

        data.dev_track_mode.prev_distance = d;
    }
    else if (mode == MODE_DEV_TRACK_DISPLAY)
    {
        int displacement = data.dev_track_mode.prev_distance - data.dev_track_mode.start_distance;
        int velocity = myAbs(displacement) / (data.dev_track_mode.delta_time / 1000.0);
        printLCD(displacement, velocity);

        if (button_pressed == 'S')
        {
            modeswitch(MODE_DEV_MENU);
        }
    }
    else if (mode == MODE_PC_PASSCODE)
    {
        // display passcode on the screen
        printLCD(0, 0);

        // compare serial input to passcode
        if (Serial.available())
        {
            char serialin[7];
            readSerialString(serialin, 7);
            if (atol(serialin) == passwords.pcpw)
            {
                modeswitch(MODE_PC);
                Serial.println(F("PC Mode Activated"));
            }
            else Serial.println(F("Incorrect Passcode"));
        }

        if (counter >= data.pc_mode_data.timer) // count down 30s timer
        {
            modeswitch(MODE_DEV_MENU);
            Serial.println(F("No Passcode Inputted"));
        }

        if (button_pressed == 'S')
        {
            modeswitch(MODE_DEV_MENU);
            Serial.println(F("Connection Cancelled"));
        }

    }
    else if (mode == MODE_PC)
    {
        printLCD(0, 0);

        if (isCommand(command, CMD_UTIL_Clock))
        {
            int mins = counter / 1000 / 60;
            int secs = (counter / 1000) % 60;
            char t[6];
            snprintf(t, 6, "%02d:%02d", mins, secs);
            Serial.println(t);
        }
        else if (isCommand(command, CMD_UTIL_SID))
        {
            Serial.println(F("12883423"));
        }
        else if (isCommand(command, CMD_UTIL_BPW))
        {
            Serial.println(passwords.userpw);
        }
        else if (isCommand(command, CMD_UTIL_APW))
        {
            Serial.println(passwords.adminpw);
        }
        else if (isCommand(command, CMD_UTIL_DPW))
        {
            Serial.println(passwords.devpw);
        }
        else if (isCommand(command, CMD_UTIL_BPW_RESET) || isCommand(command, CMD_UTIL_APW_RESET) || isCommand(command, CMD_UTIL_DPW_RESET))
        {
            Serial.println(F("Please Input New Password"));
            char newpw[50];
            while (true)
            {
                if (Serial.available() > 0)
                {
                    readSerialString(newpw, 50);
                    if (validatePassword((const char *) &newpw))
                    {
                        Serial.println(F("Successfully Set New Password"));
                        if (isCommand(command, CMD_UTIL_BPW_RESET)) strcpy(passwords.userpw, newpw);
                        else if (isCommand(command, CMD_UTIL_APW_RESET)) strcpy(passwords.adminpw, newpw);
                        else strcpy(passwords.devpw, newpw);
                        break;
                    }
                    else
                    {
                        Serial.println(F("Invalid Password, Please Enter Valid Password"));
                    }
                }
            }
        }
        else if (isCommand(command, CMD_BASIC_DCO)) modeswitch(MODE_USER_CLOSEST);
        else if (isCommand(command, CMD_BASIC_SEC)) modeswitch(MODE_USER_MAP);
        else if (isCommand(command, CMD_BASIC_SM)) modeswitch(MODE_USER_SEC);
        else if (isCommand(command, CMD_ADMIN_SOC)) modeswitch(MODE_PC_ORIGIN);
        else if (isCommand(command, CMD_ADMIN_ASEC)) 
        {
            modeswitch(MODE_ADMIN_MAP_MENU);
            Serial.println(F("Enter number of revolutions (1-9):"));
        }
        else if (isCommand(command, CMD_ADMIN_SM)) modeswitch(MODE_ADMIN_SEC);
        else if (isCommand(command, CMD_ADMIN_TSM)) 
        {
            modeswitch(MODE_ADMIN_TIMED_MENU);
            Serial.println(F("Enter time to run for (mm:ss):"));
        }
        else if (isCommand(command, CMD_PC_MAP)) modeswitch(MODE_PC_MAP);
        else if (isCommand(command, CMD_PC_CHANGE)) modeswitch(MODE_PC_CHANGE);
        else if (isCommand(command, CMD_PC_LOCAL)) modeswitch(MODE_PC_LOCAL);
        else if (isCommand(command, CMD_EXIT))
        {
            Serial.println(F("Exited PC mode"));
            pc_mode = false;
            modeswitch(MODE_DEV_MENU);
        }

        strcpy(command, "");

    }
    else if (mode == MODE_PC_MAP)
    {
        printLCD(0, 0);
        Serial.println(F("Mapping area..."));
        mapArea(3);

        createMap(dist);
        memcpy(mapped_dist, dist, READINGS_PER_REV); // store mapped distance array

        modeswitch(MODE_PC);
    }
    else if (mode == MODE_PC_CHANGE)
    {
        printLCD(0, 0);
        Serial.println(F("Detecting changes..."));
        if (mapped_dist[0] != 0)
        {
            mapArea(3);
            mapChanges(dist, mapped_dist); // only map changes when a map has been previously made
        }
        else Serial.println(F("Create a map using MODE_PC_MAP first"));
        modeswitch(MODE_PC);
    }
    else if (mode == MODE_PC_LOCAL)
    {
        printLCD(0, 0);
        Serial.println(F("Localising sensor..."));
        if (mapped_dist[0] != 0)
        {
            mapArea(3);
            localiseSensor(dist, mapped_dist);
        }
        else Serial.println(F("Create a map using MODE_PC_MAP first"));
        modeswitch(MODE_PC);
    }
    else if (mode == MODE_PC_ORIGIN)
    {
        printLCD(0,0);
        if (strstr(command, "ROT_") == command) // ROT_ at start of string
        {
            char * ptr;
            int deg = 0;
            deg = strtol(command + 4, &ptr, 10); // number of degrees

            if (deg != 0) // deg has been found in string
            {
                if (strcmp(ptr, "_CW") == 0)
                {
                    // rotate deg cw
                    stepper_counter = (int) (STEPS_PER_REV * deg / 360);
                    stepperdir = CW;
                    OCR2A = 15625 / stepperspeed;
                    bit_set(TIMSK2, OCIE2A); // enable stepper
                    while (bit_check(TIMSK2, OCIE2A)); // wait for motor to finish rotating
                }
                else if (strcmp(ptr, "_CCW") == 0)
                {
                    // rotate deg ccw
                    stepper_counter = (int) (STEPS_PER_REV * deg / 360);
                    stepperdir = CCW;
                    OCR2A = 15625 / stepperspeed;
                    bit_set(TIMSK2, OCIE2A); // enable stepper
                    while (bit_check(TIMSK2, OCIE2A)); // wait for motor to finish rotating
                }
            }
            strcpy(command, "");
        }
        else if (isCommand(command, CMD_SUBMIT))
        {
            
            Serial.println(F("Origin angle set"));
            origin = current_step;
            origin_set = true;
            
            strcpy(command, "");
            
            if (prev_mode == MODE_ADMIN_SEC ||  prev_mode == MODE_ADMIN_TIMED)
            {
                modeswitch(prev_mode);
            }
            else modeswitch(MODE_PC);
        }
    }

    if (pc_mode)
    {
        // update the command and process cancel command
        if (Serial.available())
        {
            readSerialString(command, 20);
            if (isCommand(command, CMD_CANCEL))
            {
                Serial.println(F("Operation cancelled"));
                modeswitch(MODE_PC);
            }
        }
    }

    button_pressed = 'O'; // assume button press has been acted on

    myDelay(20);
}

void setupIO()
{
    // setup motor pins
    bit_set(DDRD, 2);
    bit_set(DDRD, 3);
    bit_set(DDRB, 3);
    bit_set(DDRB, 4);
}

void setupADC()
{
    // setup adc
    bit_set(ADMUX, REFS0); // high reference voltage is vcc
    ADCSRA |= (_BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2)); // set divisor to 128
    bit_set(ADCSRA, ADEN); // enable adc
}

void setupLCD()
{
    // setup lcd - must be done before timer for some reason...
    lcd.begin(16, 2);
    delay(100);
}

void setupTimer1()
{
    // ms timer
    TCCR1A = 0; // clear settings registers
    TCCR1B = 0;

    bit_set(TCCR1A, COM1A1); // ctc mode
    TCCR1B |= (_BV(WGM12) | _BV(CS11) | _BV(CS10)); // ctc mode, 64 prescaler

    OCR1A = 250;
    bit_set(TIMSK1, OCIE1A); // interrupt on compare match A
}

void setupTimer2()
{
    // stepper timer
    TCCR2A = 0; // clear settings registers
    TCCR2B = 0;

    bit_set(TCCR2A, WGM21); // set to clear timer on compare with ocra

    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 1024 prescaler

    OCR2A = 125; // set compare to min speed
}

static char line1[LCD_LEN];
static char line2[LCD_LEN];

void printLCD(int arg1, int arg2)
{
    strcpy(line1, line_blank);
    strcpy(line2, line_blank);

    if (mode == MODE_START)
    {
        int mins = counter / 60000;
        int secs = (counter / 1000) % 60;

        snprintf(line1, LCD_LEN, "%02d:%02d           ", mins, secs);
        snprintf(line2, LCD_LEN, pw);

        if ((counter / 500) % 2 == 0)
        {
            // blink selected letter
            line2[lcdidx] = ' ';
        }
    }
    else if (mode == MODE_INVALID)
    {
        int mins = counter / 60000;
        int secs = (counter / 1000) % 60;

        snprintf(line1, LCD_LEN, "%02d:%02d           ", mins, secs);
        snprintf(line2, LCD_LEN, "INVALID");
    }
    else if (mode == MODE_USER_MENU)
    {
        snprintf(line1, LCD_LEN, "USER MODE        ");
        char options[][16] = {"Closest", "Map", "Security", "Exit"};

        // print the option selected and arrows showing if there are more options
        if (menu_selected <= 0)
        {
            // we can go right only
            snprintf(line2, LCD_LEN, "%s                ", options[menu_selected]);
            line2[15] = 0b01111110;
        }
        else if (menu_selected >= 3)
        {
            // we can go left only
            snprintf(line2, LCD_LEN, "%c %s                ", 0b01111111, options[menu_selected]);
        }
        else
        {
            snprintf(line2, LCD_LEN, "%c %s                ", 0b01111111, options[menu_selected]);
            line2[15] = 0b01111110;
        }
    }
    else if (mode == MODE_USER_CLOSEST || mode == MODE_USER_LOCK_CLOSEST)
    {
        snprintf(line1, LCD_LEN, "Detect Closest   ");
        if (mode == MODE_USER_CLOSEST)
        {
            snprintf(line2, LCD_LEN, "Scanning...       ");
        }
        else
        {
            snprintf(line2, LCD_LEN, "Closest at %dcm   ", dist[minArray(dist, READINGS_PER_REV)]);
        }
    }
    else if (mode == MODE_USER_MAP || mode == MODE_ADMIN_MAP)
    {
        int total_time = arg1;
        int elapsed_time = arg2;

        snprintf(line1, LCD_LEN, "Env Calibration     ");
        snprintf(line2, LCD_LEN, "%d%%  Time: %ds    ", (int)(100 * ((float)elapsed_time / total_time)), 1 + (total_time - elapsed_time) / 1000);
    }
    else if (mode == MODE_USER_SEC || mode == MODE_ADMIN_SEC)
    {
        snprintf(line1, LCD_LEN, "Security Mode       ");
        if (mapping)
        {
            // not calibrated yet
            snprintf(line2, LCD_LEN, "Calibrating...     ");
        }
        else snprintf(line2, LCD_LEN, "Safe                ");
    }
    else if (mode == MODE_ADMIN_TIMED)
    {
        long remaining_time = (data.admin_timed_mode.reset_time - counter) / 1000;
        if (remaining_time < 0) remaining_time = 0;
        snprintf(line1, LCD_LEN, "Timed sec mode      ");
        snprintf(line2, LCD_LEN, "Safe %02ld:%02ld", remaining_time / 60, remaining_time % 60);
    }
    else if (mode == MODE_USER_INTRUDER || mode == MODE_ADMIN_INTRUDER)
    {
        int angle = 360 * intruder / READINGS_PER_REV;
        if ((counter / 500) % 2 == 0)
        {
            snprintf(line1, LCD_LEN, "Intruder           ");
        }
        else
        {
            snprintf(line1, LCD_LEN, "                   ");
        }

        if (mode == MODE_USER_INTRUDER)
        {
            snprintf(line2, LCD_LEN, "%d cm               ", dist[intruder]);
        }
        else
        {
            snprintf(line2, LCD_LEN, "%d cm  %d deg          ", dist[intruder], angle);
        }
    }
    else if (mode == MODE_ADMIN_MENU)
    {
        snprintf(line1, LCD_LEN, "ADMIN MODE         ");
        char options[][16] = {"Origin", "Map", "Security", "Timed", "Exit"};

        if (menu_selected <= 0)
        {
            // we can go right only
            snprintf(line2, LCD_LEN, "%s                ", options[menu_selected]);
            line2[15] = 0b01111110;
        }
        else if (menu_selected >= 4)
        {
            // we can go left only
            snprintf(line2, LCD_LEN, "%c %s                ", 0b01111111, options[menu_selected]);
        }
        else
        {
            snprintf(line2, LCD_LEN, "%c %s                ", 0b01111111, options[menu_selected]);
            line2[15] = 0b01111110;
        }
    }
    else if (mode == MODE_ADMIN_ORIGIN || mode == MODE_PC_ORIGIN)
    {
        int angle = (int)(360.0 * (current_step - origin) / STEPS_PER_REV);
        if (angle < 0) angle = 360 + angle; // if angle is negative, wrap around
        snprintf(line1, LCD_LEN, "Set origin              ");
        snprintf(line2, LCD_LEN, "%d degrees              ", angle);
    }
    else if (mode == MODE_ADMIN_MAP_MENU)
    {
        snprintf(line1, LCD_LEN, "Env Calibration     ");
        snprintf(line2, LCD_LEN, "%d revolutions      ", revolutions);
    }
    else if (mode == MODE_ADMIN_TIMED_MENU)
    {
        snprintf(line1, LCD_LEN, "Timed sec mode     ");
        if (!pc_mode)
        {
            snprintf(line2, LCD_LEN, "%02ld:%02ld           ", data.admin_timed_mode.timer / 60, data.admin_timed_mode.timer % 60);
            if ((counter / 500) % 2 == 0)
            {
                if (lcdidx < 2) line2[lcdidx] = ' ';
                else line2[lcdidx + 1] = ' ';
            }
        }
    }
    else if (mode == MODE_DEV_MENU)
    {
        snprintf(line1, LCD_LEN, "DEV MODE             ");
        char options[][16] = {"PC Mode", "Stepper", "Find Object", "Track Object", "Exit"};

        if (menu_selected <= 0)
        {
            // we can go right only
            snprintf(line2, LCD_LEN, "%s                ", options[menu_selected]);
            line2[15] = 0b01111110;
        }
        else if (menu_selected >= 4)
        {
            // we can go left only
            snprintf(line2, LCD_LEN, "%c %s                ", 0b01111111, options[menu_selected]);
        }
        else
        {
            snprintf(line2, LCD_LEN, "%c %s                ", 0b01111111, options[menu_selected]);
            line2[15] = 0b01111110;
        }
    }
    else if (mode == MODE_DEV_STEPPER)
    {
        int d = arg1;
        int angle = (int)(360.0 * (current_step - origin) / STEPS_PER_REV);
        if (angle < 0) angle = 360 + angle;

        snprintf(line1, LCD_LEN, "Stepper test       ");
        snprintf(line2, LCD_LEN, "%d deg, %d cm      ", angle, d);
    }
    else if (mode == MODE_DEV_FIND_MENU)
    {
        snprintf(line1, LCD_LEN, "Object detection   ");
        snprintf(line2, LCD_LEN, "%d cm              ", data.dev_find_mode.find_distance);
    }
    else if (mode == MODE_DEV_FIND)
    {
        snprintf(line1, LCD_LEN, "Object detection   ");
        snprintf(line1, LCD_LEN, "Locating object... ");
    }
    else if (mode == MODE_DEV_LOCK_FIND)
    {
        snprintf(line1, LCD_LEN, "Object detection   ");
        snprintf(line2, LCD_LEN, "Found %d cm        ", arg1);
    }
    else if (mode == MODE_DEV_TRACK_MENU)
    {
        snprintf(line1, LCD_LEN, "Press select to    ");
        snprintf(line2, LCD_LEN, "start tracking     ");
    }
    else if (mode == MODE_DEV_TRACK)
    {
        snprintf(line1, LCD_LEN, "Tracking mode      ");
        snprintf(line2, LCD_LEN, "Object detected    ");
    }
    else if (mode == MODE_DEV_TRACK_DISPLAY)
    {
        int displacement = arg1;
        int velocity = arg2;
        snprintf(line1, LCD_LEN, "Dist: %d cm       ", displacement);
        snprintf(line2, LCD_LEN, "Vel: %d cm/s      ", velocity);
    }
    else if (mode == MODE_LOCKOUT)
    {
        for (int i = 0; i < 16; ++i)
        {
            line1[i] = 255;
            line2[i] = 255;
        }
    }
    else if (mode == MODE_PC_PASSCODE)
    {
        int time_remaining = (data.pc_mode_data.timer - counter) / 1000;
        snprintf(line1, LCD_LEN, "Passcode: %06ld ", passwords.pcpw);
        snprintf(line2, LCD_LEN, "Time: %ds       ", time_remaining);
    }
    else if (mode == MODE_PC || mode == MODE_PC_MAP || mode == MODE_PC_CHANGE)
    {
        snprintf(line1, LCD_LEN, "PC Mode          ");
    }

    if (pc_mode)
    {
        // check if the lines have changed and print if they have
        if (strcmp(line1, oldline1) != 0)
        {
            Serial.print("| ");
            Serial.print(line1);
            Serial.println(" |");
        }
        if (strcmp(line2, oldline2) != 0)
        {
            Serial.print("| ");
            Serial.print(line2);
            Serial.println(" |");
        }

        strcpy(oldline1, line1);
        strcpy(oldline2, line2);
    }

    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}

void modeswitch(int nextmode)
{
    prev_mode = mode;
    menu_selected = 0; // always reset the menu when switching modes
    lcdidx = 0;
    if (nextmode == MODE_START)
    {
        memset(dist, 0, sizeof(dist)); // reset distance measuring array
        bit_clear(TIMSK2, OCIE2A); // disable stepper
        strcpy(pw, "________________");
    }
    else if (nextmode == MODE_INVALID)
    {
        data.invalid_mode.timetoreset = counter + 5000;
        attempts += 1;
        if (attempts >= 5)
        {
            nextmode = MODE_LOCKOUT;
        }
    }
    else if (nextmode == MODE_USER_CLOSEST)
    {
        stepperdir = CW; // start off clockwise
    }
    else if (nextmode == MODE_USER_LOCK_CLOSEST)
    {
        // find the minimum value and step to it, then change mode
        int minidx = minArray(dist, READINGS_PER_REV);
        int goal_step = ((STEPS_PER_REV / READINGS_PER_REV) * minidx) + start_step;
        goToStep(goal_step);
    }
    else if (nextmode == MODE_USER_INTRUDER || nextmode == MODE_ADMIN_INTRUDER)
    {
        int goal_step = ((STEPS_PER_REV / READINGS_PER_REV) * intruder) + start_step + origin;
        goToStep(goal_step);
    }
    else if (nextmode == MODE_ADMIN_ORIGIN)
    {
        memset(dist, 0, sizeof(dist)); // reset distance measuring array
        stepperspeed = 1000;
        OCR2A = 15625 / stepperspeed;
        bit_clear(TIMSK2, OCIE2A); // intially disable stepper
    }
    else if (nextmode == MODE_DEV_STEPPER)
    {
        stepperspeed = 1000;
        OCR2A = 15625 / stepperspeed;
    }
    else if (nextmode == MODE_ADMIN_MENU)
    {
        goToStep(origin); // return to origin angle
    }
    else if (mode == MODE_ADMIN_TIMED_MENU)
    {
        data.admin_timed_mode.timer = 0;
    }
    else if (nextmode == MODE_DEV_FIND_MENU)
    {
        data.dev_find_mode.find_distance = 30;
    }
    else if (nextmode == MODE_DEV_LOCK_FIND)
    {
        uint8_t diffs[READINGS_PER_REV];
        for (int i = 0; i < READINGS_PER_REV; ++i)
        {
            diffs[i] = myAbs(dist[i] - data.dev_find_mode.find_distance);
        }
        int minidx = minArray(diffs, READINGS_PER_REV);
        int goal_step = ((STEPS_PER_REV / READINGS_PER_REV) * minidx) + start_step;
        goToStep(goal_step);
    }
    else if (nextmode == MODE_PC_PASSCODE)
    {
        passwords.pcpw = generatePasscode();
        Serial.println(F("Please Input Passcode:"));
        data.pc_mode_data.timer = counter + 30 * 1000;
    }
    else if (nextmode == MODE_PC)
    {
        goToStep(origin); // return to origin angle
        pc_mode = true;
    }
    else if (nextmode == MODE_PC_ORIGIN)
    {
        memset(dist, 0, sizeof(dist)); // reset distance measuring array
        Serial.println(F("Input origin angle command:"));
    }
    mode = nextmode;
}

ISR(TIMER1_COMPA_vect) // ms timer
{
    // read adc from distance sensor
    if (counter % 40 == 0)
    {
        bit_set(ADMUX, MUX0); // select A1
        bit_set(ADCSRA, ADSC); // intialise conversion
        while ((ADCSRA & 1 << ADSC)); // wait until conversion is complete

        adcarr[iarr] = ADC; // add adc value to averaging array
        iarr++;
        if (iarr >= NUMAVG)
        {
            iarr = 0;

            // calculate median and set global distance reading
            int mean = medianArray(adcarr, NUMAVG);
            float distance = adc2distance(mean);
            if ((int)(distance * 100) % 100 < 50)
            {
                // round down
                current_distance = (uint8_t) distance;
            }
            else
            {
                // round up
                current_distance = (uint8_t) distance + 1;
            }
        }
    }

    // read button
    if (counter % 10 == 1)
    {
        bit_clear(ADMUX, MUX0); // select A0
        bit_set(ADCSRA, ADSC); // intialise conversion
        while ((ADCSRA & 1 << ADSC)); // wait until conversion is complete

        int x = ADC;
        char current_button;

        if (x < 60) {
            // right
            current_button = 'R';
        }
        else if (x < 200) {
            // up
            current_button = 'U';
        }
        else if (x < 400) {
            // down
            current_button = 'D';
        }
        else if (x < 600) {
            // left
            current_button = 'L';
        }
        else if (x < 800) {
            // select
            current_button = 'S';
        }
        else
        {
            current_button = 'O'; // off, no button pressed
        }
        if (current_button != button)
        {
            button = current_button; // update stored value for button
            if (current_button != 'O') button_pressed = current_button; // set button_pressed flag if a button has been pressed
        }
    }

    counter++;

    if (countdown_enabled)
    {
        countdown_ms--;
        if (countdown_ms == 0)
        {
            countdown_enabled = false;
        }
    }
}

ISR(TIMER2_COMPA_vect) // stepper timer
{
    //uint8_t x;
    // take a step
    switch (current_step % 8) // each time, clear all pins, set next active pin
    {
        case 0:
            PORTD = ((PORTD & 0b11110011) | _BV(IN1));
            PORTB &= 0b11100111; // clear pins 11,12
            break;
        case 1:
            PORTD = ((PORTD & 0b11110011) | _BV(IN1) | _BV(IN2));
            PORTB &= 0b11100111; // clear pins 11,12
            break;
        case 2:
            PORTD = ((PORTD & 0b11110011) | _BV(IN2));
            PORTB &= 0b11100111;
            break;
        case 3:
            PORTD = ((PORTD & 0b11110011) | _BV(IN2));
            PORTB = ((PORTB & 0b11100111) | _BV(IN3));
            break;
        case 4:
            PORTD &= 0b11110011;
            PORTB = ((PORTB & 0b11100111) | _BV(IN3));
            break;
        case 5:
            PORTD &= 0b11110011;
            PORTB = ((PORTB & 0b11100111) | _BV(IN3) | _BV(IN4));
            break;
        case 6:
            PORTD &= 0b11110011;
            PORTB = ((PORTB & 0b11100111) | _BV(IN4));
            break;
        case 7:
            PORTD = ((PORTD & 0b11110011) | _BV(IN1));
            PORTB = ((PORTB & 0b11100111) | _BV(IN4));
            break;
    }
    if (stepperdir == CCW)
    {
        current_step++;
        if (current_step > 4095)
        {
            current_step = 0;
        }
    }
    else if (stepperdir == CW)
    {
        current_step--;
        if (current_step < 0)
        {
            current_step = 4095;
        }
    }

    if (mode != MODE_ADMIN_ORIGIN && mode != MODE_DEV_STEPPER) // don't count down in these modes
    {
        stepper_counter--;
        if (stepper_counter <= 0)
        {
            bit_clear(TIMSK2, OCIE2A); // disable interrupt for this timer
        }
    }
}

void recognisePassword(char password[])
{
    if (strstr((const char*) password, passwords.userpw))
    {
        modeswitch(MODE_USER_MENU);
    }
    else if (strstr((const char*)password, passwords.adminpw))
    {
        modeswitch(MODE_ADMIN_MENU);
    }
    else if (strstr((const char*)password, passwords.devpw))
    {
        modeswitch(MODE_DEV_MENU);
    }
    else
    {
        modeswitch(MODE_INVALID);
    }
}

float adc2v(int adc)
{
    float v = 5 * adc / 1024.0;
    return v;
}

float adc2distance(long a)
{
    // trendline for voltage to distance curve from datasheet
    //float d = 32.4*v*v*v*v - 227.16*v*v*v + 587.17*v*v - 689.95*v + 359.06;

    double c1 = 1.728e-08;
    double c2 = -2.379e-05;
    double c3 = 0.01207;
    double c4 = -2.813;
    double c5 = 301.4;

    double d = (c1 * a * a * a * a) + (c2 * a * a * a) + (c3 * a * a) + (c4 * a) + c5;

    if (d > 150)
    {
        d = 150;
    }
    else if (d < 20)
    {
        d = 20;
    }

    return (float)d;
}

long avgArray(int a[], int sz)
{
    long total = 0;
    for (int i = 0; i < sz; ++i)
    {
        total += a[i];
    }
    total /= sz;
    return total;
}

int medianArray(int a[], int sz)
{
    // sort the array
    int temp = 0;
    for (int i = 0 ; i < sz ; i++)
    {
        for (int j = 0 ; j < sz - 1 ; j++)
        {
            if (a[j] > a[j + 1])
            {
                temp = a[j];
                a[j] = a[j + 1];
                a[j + 1]  = temp;
            }
        }
    }

    return a[sz / 2];
}

uint8_t minArray(uint8_t a[], int sz)
{
    int minidx = 0;
    for (int i = 1; i < sz; ++i)
    {
        if (a[i] < a[minidx])
        {
            minidx = i;
        }
    }
    return minidx; // return the index of the minimum element
}

// takes scans of the area and averages based on number of scans
void mapArea(int scans)
{
    for (int i = 0; i < READINGS_PER_REV; ++i)
    {
        // clear distance array
        dist[i] = 0;
    }

    takeScans(dist, scans);
}

void takeScans(uint8_t output[], int scans)
{
    long scan_time = READINGS_PER_REV * NUMAVG * 40L + 1000 * STEPS_PER_REV / stepperspeed;
    long total_time = scans * scan_time;
    uint16_t intermediate_output[READINGS_PER_REV] = {0}; // use an intermediate to avoid overflow

    for (int scan = 0; scan < scans; scan++)
    {
        // get distance at intervals
        for (int i = 0; i < READINGS_PER_REV; ++i)
        {
            printLCD(total_time, scan * scan_time + i * scan_time / READINGS_PER_REV); // print lcd from here since we need time remaining

            uint8_t d = readDistanceSensor();

            int idx;
            if (stepperdir == CCW)
            {
                idx = i;
            }
            else // when going clockwise, place in distance array in reverse order
            {
                idx = (READINGS_PER_REV - i) % READINGS_PER_REV;
                // mod instead of subtracting 1 to prevent an off by 1 error
            }

            intermediate_output[idx] += d;

            stepper_counter = STEPS_PER_REV / READINGS_PER_REV; // number of steps in 1 interval
            OCR2A = 15625 / stepperspeed;
            bit_set(TIMSK2, OCIE2A); // enable stepper
            while (bit_check(TIMSK2, OCIE2A)); // wait for motor to finish rotating
        }

        stepperdir = !stepperdir; // reverse direction so cord doesn't tangle
    }
    // divide by number of scans and copy into output array
    for (int i = 0; i < READINGS_PER_REV; ++i)
    {
        output[i] = intermediate_output[i] / scans;
    }
}

void goToStep(int goal_step)
{
    // we have current_step and s
    // want to find the number of steps and direction to go
    int steps_left = myAbs(goal_step - current_step);
    int half_rev = STEPS_PER_REV / 2;

    if (goal_step >= current_step && steps_left <= half_rev)
    {
        stepper_counter = steps_left;
        stepperdir = CCW;
    }
    else if (goal_step < current_step && steps_left <= half_rev)
    {
        stepper_counter = steps_left;
        stepperdir = CW;
    }
    else if (goal_step >= current_step && steps_left > half_rev)
    {
        stepper_counter = STEPS_PER_REV - steps_left;
        stepperdir = CW;
    }
    else if (goal_step < current_step && steps_left > half_rev)
    {
        stepper_counter = STEPS_PER_REV - steps_left;
        stepperdir = CCW;
    }

    stepperspeed = 1000;
    OCR2A = 15625 / stepperspeed;
    bit_set(TIMSK2, OCIE2A); // enable stepper
}

uint8_t readDistanceSensor()
{
    // wait for distance to be read
    long wait_time = (NUMAVG + 1) * 40;
    myDelay(wait_time);

    return current_distance;
}

int myAbs(int n)
{
    if (n > 0)
    {
        return n;
    }
    else return n * -1;
}

long generatePasscode()
{
    long total = 0;
    srand(int(counter % 1000)); // use ms to seed
    for (int i = 0; i < 6; ++i)
    {
        int r = rand() % 10;
        total += (r * myPow(10, i));
    }
    return total;
}

long myPow(int base, int power)
{
    long total = 1;
    for (int i = 0; i < power; ++i)
    {
        total *= base;
    }
    return total;
}

bool validatePassword(const char *pw)
{
    if (strlen(pw) > 16)
    {
        return false; // validate length
    }

    while (*pw)
    {
        if (*pw < '0' || *pw > '9')
        {
            return false;
        }
        pw++;
    }
    return true;
}

void createMap(uint8_t distance[])
{
    // iterate over each cell in map, index in row, column
    for (int row = 0; row < MAP_SIZE; row++)
    {
        for (int col = 0; col < MAP_SIZE; col++)
        {
            // find distance and angle
            uint8_t centre = MAP_SIZE / 2;
            uint8_t d_to_centre = (mySqrt((row - centre) * (row - centre) + (col - centre) * (col - centre))) * CM_PER_CELL;
            float angle_to_centre = atan((float)(col - centre) / (float)(row - centre));

            int segment;

            if ((row - centre) < 0 && (col - centre) <= 0)
            {

            }
            else if ((row - centre) >= 0 && (col - centre) <= 0)
            {
                angle_to_centre += PI;
            }
            else if ((row - centre) >= 0 && (col - centre) > 0)
            {
                angle_to_centre += PI;
            }
            else if ((row - centre) < 0 && (col - centre) > 0)
            {
                angle_to_centre += 2 * PI;
            }

            char map_val = '0';
            segment = myRound(angle_to_centre * READINGS_PER_REV / (2 * PI)) % READINGS_PER_REV;
            if (row == centre && col == centre) // sensor location
            {
                map_val = '1';
            }
            else if (d_to_centre > 100 || d_to_centre <= 20) // out of range
            {
                map_val = '2';
            }
            else if (distance[segment] < d_to_centre) // object present
            {
                map_val = '9';
            }
            Serial.print(map_val);
            Serial.print("  ");
        }
        Serial.println();
    }
}

int myRound(float num)
{
    if (num >= (int) num + 0.5)
    {
        return ((int) num + 1);
    }
    else return (int) num;
}

float mySqrt(int b)
{
    float x0 = 50; // initial guess
    float x1 = 0;
    int maxiterations = 10;

    for (int i = 0; i < maxiterations; ++i)
    {
        x1 = x0 - (x0 * x0 - b) / (2 * x0);
        if (myAbs(x1 - x0) < 0.01)
        {
            break;
        }

        x0 = x1;
    }
    return x1;
}

void mapChanges(uint8_t newdist[], uint8_t olddist[])
{
    // compare new and old distance and identify differences
    bool changed[READINGS_PER_REV] = {false};
    for (int i = 0; i < READINGS_PER_REV; ++i)
    {
        if ((olddist[i] - newdist[i]) > 10)
        {
            changed[i] = true;

        }
    }

    // iterate over each cell in map, index in row, column
    for (int row = 0; row < MAP_SIZE; row++)
    {
        for (int col = 0; col < MAP_SIZE; col++)
        {
            // find distance and angle
            uint8_t centre = MAP_SIZE / 2;
            uint8_t d_to_centre = (mySqrt((row - centre) * (row - centre) + (col - centre) * (col - centre))) * CM_PER_CELL;
            float angle_to_centre = atan((float)(col - centre) / (float)(row - centre));

            int segment;

            if ((row - centre) < 0 && (col - centre) <= 0)
            {

            }
            else if ((row - centre) >= 0 && (col - centre) <= 0)
            {
                angle_to_centre += PI;
            }
            else if ((row - centre) >= 0 && (col - centre) > 0)
            {
                angle_to_centre += PI;
            }
            else if ((row - centre) < 0 && (col - centre) > 0)
            {
                angle_to_centre += 2 * PI;
            }

            char map_val = '0';
            segment = myRound(angle_to_centre * READINGS_PER_REV / (2 * PI)) % READINGS_PER_REV;
            if (row == centre && col == centre) // sensor location
            {
                map_val = '1';
            }
            else if (d_to_centre > 100 || d_to_centre <= 20) // out of range
            {
                map_val = '2';
            }
            else if (newdist[segment] < d_to_centre) // object present
            {
                if (changed[segment])
                {
                    map_val = '7';
                }
                else map_val = '9';
            }
            Serial.print(map_val);
            Serial.print("  ");
        }
        Serial.println();
    }
}

void localiseSensor(uint8_t newdist[], uint8_t olddist[])
{
    uint8_t seg_length = READINGS_PER_REV / 4;
    uint8_t segments[] = {seg_length - (seg_length / 2), seg_length * 2 - (seg_length / 2), seg_length * 3 - (seg_length / 2), seg_length * 4 - (seg_length / 2)};

    // look at top and bottom segments and determine delta y
    int delta_y = 0;
    int counted = 0;
    for (int i = 0; i < seg_length; ++i)
    {
        // top
        if (newdist[(i + segments[3]) % READINGS_PER_REV] < 150 && olddist[(i + segments[3]) % READINGS_PER_REV] < 150)
        {
            // in range
            delta_y -= newdist[(i + segments[3]) % READINGS_PER_REV] - olddist[(i + segments[3]) % READINGS_PER_REV];
            counted ++;
        }

        // bottom
        if (newdist[i + segments[1]] < 150 && olddist[i + segments[1]] < 150)
        {
            delta_y += newdist[i + segments[1]] - olddist[i + segments[1]];
            counted ++;
        }
    }
    delta_y /= counted;

    // look at left and right segments and determine delta x
    int delta_x = 0;
    counted = 0;
    for (int i = 0; i < seg_length; ++i)
    {
        // left
        if (newdist[i + segments[0]] < 150 && olddist[i + segments[0]] < 150)
        {
            delta_x += newdist[i + segments[0]] - olddist[i + segments[0]];
            counted ++;
        }

        // right
        if (newdist[i + segments[2]] < 150 && olddist[i + segments[2]] < 150)
        {
            delta_x -= newdist[i + segments[2]] - olddist[i + segments[2]];
            counted ++;
        }
    }
    delta_x /= counted;

    

    Serial.print(F("New grid coords (sensor at 0, 0): "));
    Serial.print(myRound(((float)delta_x)/CM_PER_CELL));
    Serial.print(", ");
    Serial.println(myRound(((float)delta_y)/CM_PER_CELL));
}

void myDelay(long delay_ms)
{
    countdown_ms = delay_ms;
    countdown_enabled = true;
    while (countdown_enabled);
}
